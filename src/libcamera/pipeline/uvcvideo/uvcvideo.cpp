/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * uvcvideo.cpp - Pipeline handler for uvcvideo devices
 */

#include <algorithm>
#include <deque>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <memory>
#include <sys/mman.h>
#include <tuple>

#include <linux/uvcvideo.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/sysfs.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(UVC)

/*
 * The UVCH buffer contains an unsigned char array encoding UVC
 * timing data that needs to be recast into usable data. The contents
 * of that array are specified in the UVC specifications manual, and
 * are copied here exactly as they are specified.
 */
struct UVCTimingBuf {
	unsigned int pts; /* Presentation TimeStamp, in device clock time*/
	unsigned int stc; /* Source Time Clock, in device clock time*/
	unsigned short sofDevice; /* Start Of Frame, in usb frame number (clock) units*/
} __attribute__((packed));

/* Raw timestamp input to the timestamp calculation function. */
struct UVCTimestampData {
	unsigned long long tsHost; /* System clock timestamp in nanoseconds*/
	unsigned short sofHost; /* The usb clock at the time tsHost was taken*/
	unsigned int stcDevice; /* The UVC device source timestamp */
	unsigned short sofDevice; /* the usb clock at the time the STC was taken*/

	/* presentation time stamp to be converted into a system clock timestamp */
	unsigned int ptsDevice;
};

class UVCCameraData : public Camera::Private
{
public:
	UVCCameraData(PipelineHandler *pipe)
		: Camera::Private(pipe)
	{
	}

	int init(MediaDevice *media);
	void addControl(uint32_t cid, const ControlInfo &v4l2info,
			ControlInfoMap::Map *ctrls);
	void bufferReady(FrameBuffer *buffer);
	void releaseMetadataBuffers();
	void releaseBuffers();
	void bufferReadyMetadata(FrameBuffer *buffer);
	void handleUnfinishedRequests();

	const std::string &id() const { return id_; }

	std::unique_ptr<V4L2VideoDevice> video_;
	std::unique_ptr<V4L2VideoDevice> metadata_;
	Stream stream_;
	std::vector<std::unique_ptr<FrameBuffer>> metadataBuffers_;
	std::vector<std::unique_ptr<uint8_t, void (*)(uint8_t *)>>
		mappedMetaAddresses_;
	std::map<PixelFormat, std::vector<SizeRange>> formats_;
	std::queue<FrameBuffer *> pendingVideoBuffers_;
	std::queue<std::tuple<unsigned int, UVCTimestampData>> pendingMetadata_;

private:
	typedef enum MatchResult { 
		MatchFound,
		MatchNotFound,
		DropDetected
	} MatchResult;

	int initMetadata(MediaDevice *media);
	void completeRequest(FrameBuffer *buffer, uint64_t timestamp);
	void endCorruptedMetadataStream();
	MatchResult FindAndCompleteBuffer();
	void addTimestampData(uvc_meta_buf &rawMetadata, UVCTimingBuf &packed);

	std::deque<UVCTimestampData> timeSamples_;
	const unsigned int frameStart_ = 1;
	const unsigned int maxVidBuffersInQueue_ = 3;
	const unsigned int bufferRingSize_ = 32;

	bool generateId();

	std::string id_;
};

class UVCCameraConfiguration : public CameraConfiguration
{
public:
	UVCCameraConfiguration(UVCCameraData *data);

	Status validate() override;

private:
	UVCCameraData *data_;
};

class PipelineHandlerUVC : public PipelineHandler
{
public:
	PipelineHandlerUVC(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	int processControl(ControlList *controls, unsigned int id,
			   const ControlValue &value);
	int processControls(UVCCameraData *data, Request *request);

	int createMetadataBuffers(Camera *camera, unsigned int count);

	UVCCameraData *cameraData(Camera *camera)
	{
		return static_cast<UVCCameraData *>(camera->_d());
	}
};

UVCCameraConfiguration::UVCCameraConfiguration(UVCCameraData *data)
	: CameraConfiguration(), data_(data)
{
}

CameraConfiguration::Status UVCCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	if (transform != Transform::Identity) {
		transform = Transform::Identity;
		status = Adjusted;
	}

	/* Cap the number of entries to the available streams. */
	if (config_.size() > 1) {
		config_.resize(1);
		status = Adjusted;
	}

	StreamConfiguration &cfg = config_[0];
	const StreamFormats &formats = cfg.formats();
	const PixelFormat pixelFormat = cfg.pixelFormat;
	const Size size = cfg.size;

	const std::vector<PixelFormat> pixelFormats = formats.pixelformats();
	auto iter = std::find(pixelFormats.begin(), pixelFormats.end(), pixelFormat);
	if (iter == pixelFormats.end()) {
		cfg.pixelFormat = pixelFormats.front();
		LOG(UVC, Debug)
			<< "Adjusting pixel format from " << pixelFormat
			<< " to " << cfg.pixelFormat;
		status = Adjusted;
	}

	const std::vector<Size> &formatSizes = formats.sizes(cfg.pixelFormat);
	cfg.size = formatSizes.front();
	for (const Size &formatsSize : formatSizes) {
		if (formatsSize > size)
			break;

		cfg.size = formatsSize;
	}

	if (cfg.size != size) {
		LOG(UVC, Debug)
			<< "Adjusting size from " << size << " to " << cfg.size;
		status = Adjusted;
	}

	cfg.bufferCount = 4;

	V4L2DeviceFormat format;
	format.fourcc = data_->video_->toV4L2PixelFormat(cfg.pixelFormat);
	format.size = cfg.size;

	int ret = data_->video_->tryFormat(&format);
	if (ret)
		return Invalid;

	cfg.stride = format.planes[0].bpl;
	cfg.frameSize = format.planes[0].size;

	if (cfg.colorSpace != format.colorSpace) {
		cfg.colorSpace = format.colorSpace;
		status = Adjusted;
	}

	return status;
}

PipelineHandlerUVC::PipelineHandlerUVC(CameraManager *manager)
	: PipelineHandler(manager)
{
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerUVC::generateConfiguration(Camera *camera,
					  Span<const StreamRole> roles)
{
	UVCCameraData *data = cameraData(camera);
	std::unique_ptr<CameraConfiguration> config =
		std::make_unique<UVCCameraConfiguration>(data);

	if (roles.empty())
		return config;

	StreamFormats formats(data->formats_);
	StreamConfiguration cfg(formats);

	cfg.pixelFormat = formats.pixelformats().front();
	cfg.size = formats.sizes(cfg.pixelFormat).back();
	cfg.bufferCount = 4;

	config->addConfiguration(cfg);

	config->validate();

	return config;
}

int PipelineHandlerUVC::configure(Camera *camera, CameraConfiguration *config)
{
	UVCCameraData *data = cameraData(camera);
	StreamConfiguration &cfg = config->at(0);
	int ret;

	V4L2DeviceFormat format;
	format.fourcc = data->video_->toV4L2PixelFormat(cfg.pixelFormat);
	format.size = cfg.size;

	ret = data->video_->setFormat(&format);
	if (ret)
		return ret;

	if (format.size != cfg.size ||
	    format.fourcc != data->video_->toV4L2PixelFormat(cfg.pixelFormat))
		return -EINVAL;

	cfg.setStream(&data->stream_);
	return 0;
}

void UVCCameraData::releaseMetadataBuffers()
{
	if (metadata_)
		metadata_->releaseBuffers();
	metadataBuffers_.clear();
	mappedMetaAddresses_.clear();
	metadata_ = nullptr;

	return;
}

void UVCCameraData::releaseBuffers()
{	
	releaseMetadataBuffers();

	video_->releaseBuffers();
	return;
}

/**
 * UVC Metadata stream does not support exporting buffers via EXPBUF,
 * so it is necessary to create and store mmap-ed addresses.
 * Metadata buffers are internal to libcamera. They are not, and
 * cannot be, exposed to the user.
 *
 * Returns the number of buffers allocated and mapped.
 *
 * \return The number of buffers allocated, or a negative error code if
 * the number of buffers allocated was not equal to "count"
 * \retval -EINVAL if "count" buffers were not successfully allocated.
 * \retval MappedFrameBuffer::error()
 */
int PipelineHandlerUVC::createMetadataBuffers(Camera *camera, unsigned int count)
{
	UVCCameraData *data = cameraData(camera);
	int ret = data->metadata_->allocateBuffers(count, &data->metadataBuffers_);
	if (ret < 0)
		return -EINVAL;

	for (unsigned int i = 0; i < count; i++) {
		std::unique_ptr<FrameBuffer> &buffer = data->metadataBuffers_[i];
		unsigned int bufferPlaneLength = buffer->planes()[0].length;
		void *address = mmap(NULL, bufferPlaneLength,
				     PROT_READ | PROT_WRITE,
				     MAP_SHARED,
				     buffer->planes()[0].fd.get(), buffer->planes()[0].offset);

		if (address == MAP_FAILED) {
			LOG(UVC, Error) << "Failed to mmap UVC metadata plane: -"
					<< strerror(errno);
			data->releaseMetadataBuffers();
			return -ENOMEM;
		}
		uint8_t *uint_addr = reinterpret_cast<uint8_t*>(address);

		data->mappedMetaAddresses_.emplace_back(
			uint_addr, [](uint8_t *p) {
                       munmap(p,sizeof(uvc_meta_buf)+ sizeof(UVCTimingBuf));
		        });

		buffer->setCookie(i);		
		data->metadataBuffers_[i]->setCookie(i);
	}
	return ret;
}

int PipelineHandlerUVC::exportFrameBuffers(Camera *camera, Stream *stream,
					   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	UVCCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	return data->video_->exportBuffers(count, buffers);
}

int PipelineHandlerUVC::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	UVCCameraData *data = cameraData(camera);
	unsigned int count = data->stream_.configuration().bufferCount;

	int ret = data->video_->importBuffers(count);
	if (ret < 0)
		return ret;

	ret = data->video_->streamOn();
	if (ret < 0) {
		data->releaseBuffers();
		return ret;
	}

	/*
	 * If metadata allocation fails, exit this function but
	 * do not return a failure as video started successfully.
	 * Fall back on using driver timestamps.
	 */
	if (data->metadata_) {
		if (createMetadataBuffers(camera, count) < 0 ||
		    data->metadata_->streamOn()) {
			LOG(UVC, Warning)
				<< "Metadata stream unavailable.  Using driver timestamps.";
			data->metadata_ = nullptr;
			return 0;
		}

		for (std::unique_ptr<FrameBuffer> &buf : data->metadataBuffers_) {
			ret = data->metadata_->queueBuffer(buf.get());
			if (ret < 0) {
				LOG(UVC, Warning)
					<< "Metadata stream unavailable.  Using driver timestamps.";
				data->releaseMetadataBuffers();
				return 0;
			}
		}
	}
	
	return 0;
}

void PipelineHandlerUVC::stopDevice(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);

	data->video_->streamOff();

	if (data->metadata_)
		data->metadata_->streamOff();

	data->handleUnfinishedRequests();
	data->releaseBuffers();
}

int PipelineHandlerUVC::processControl(ControlList *controls, unsigned int id,
				       const ControlValue &value)
{
	uint32_t cid;

	if (id == controls::Brightness)
		cid = V4L2_CID_BRIGHTNESS;
	else if (id == controls::Contrast)
		cid = V4L2_CID_CONTRAST;
	else if (id == controls::Saturation)
		cid = V4L2_CID_SATURATION;
	else if (id == controls::AeEnable)
		cid = V4L2_CID_EXPOSURE_AUTO;
	else if (id == controls::ExposureTime)
		cid = V4L2_CID_EXPOSURE_ABSOLUTE;
	else if (id == controls::AnalogueGain)
		cid = V4L2_CID_GAIN;
	else
		return -EINVAL;

	const ControlInfo &v4l2Info = controls->infoMap()->at(cid);
	int32_t min = v4l2Info.min().get<int32_t>();
	int32_t def = v4l2Info.def().get<int32_t>();
	int32_t max = v4l2Info.max().get<int32_t>();

	/*
	 * See UVCCameraData::addControl() for explanations of the different
	 * value mappings.
	 */
	switch (cid) {
	case V4L2_CID_BRIGHTNESS: {
		float scale = std::max(max - def, def - min);
		float fvalue = value.get<float>() * scale + def;
		controls->set(cid, static_cast<int32_t>(lroundf(fvalue)));
		break;
	}

	case V4L2_CID_SATURATION: {
		float scale = def - min;
		float fvalue = value.get<float>() * scale + min;
		controls->set(cid, static_cast<int32_t>(lroundf(fvalue)));
		break;
	}

	case V4L2_CID_EXPOSURE_AUTO: {
		int32_t ivalue = value.get<bool>()
			       ? V4L2_EXPOSURE_APERTURE_PRIORITY
			       : V4L2_EXPOSURE_MANUAL;
		controls->set(V4L2_CID_EXPOSURE_AUTO, ivalue);
		break;
	}

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		controls->set(cid, value.get<int32_t>() / 100);
		break;

	case V4L2_CID_CONTRAST:
	case V4L2_CID_GAIN: {
		float m = (4.0f - 1.0f) / (max - def);
		float p = 1.0f - m * def;

		if (m * min + p < 0.5f) {
			m = (1.0f - 0.5f) / (def - min);
			p = 1.0f - m * def;
		}

		float fvalue = (value.get<float>() - p) / m;
		controls->set(cid, static_cast<int32_t>(lroundf(fvalue)));
		break;
	}

	default: {
		int32_t ivalue = value.get<int32_t>();
		controls->set(cid, ivalue);
		break;
	}
	}

	return 0;
}

int PipelineHandlerUVC::processControls(UVCCameraData *data, Request *request)
{
	ControlList controls(data->video_->controls());

	for (const auto &[id, value] : request->controls())
		processControl(&controls, id, value);

	for (const auto &ctrl : controls)
		LOG(UVC, Debug)
			<< "Setting control " << utils::hex(ctrl.first)
			<< " to " << ctrl.second.toString();

	int ret = data->video_->setControls(&controls);
	if (ret) {
		LOG(UVC, Error) << "Failed to set controls: " << ret;
		return ret < 0 ? ret : -EINVAL;
	}

	return ret;
}

int PipelineHandlerUVC::queueRequestDevice(Camera *camera, Request *request)
{
	UVCCameraData *data = cameraData(camera);
	FrameBuffer *buffer = request->findBuffer(&data->stream_);
	if (!buffer) {
		LOG(UVC, Error)
			<< "Attempt to queue request with invalid stream";

		return -ENOENT;
	}

	int ret = processControls(data, request);
	if (ret < 0)
		return ret;

	ret = data->video_->queueBuffer(buffer);
	if (ret < 0)
		return ret;

	return 0;
}

bool PipelineHandlerUVC::match(DeviceEnumerator *enumerator)
{
	MediaDevice *media;
	DeviceMatch dm("uvcvideo");

	media = acquireMediaDevice(enumerator, dm);
	if (!media)
		return false;

	std::unique_ptr<UVCCameraData> data = std::make_unique<UVCCameraData>(this);

	if (data->init(media))
		return false;

	/* Create and register the camera. */
	std::string id = data->id();
	std::set<Stream *> streams{ &data->stream_ };
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(data), id, streams);
	registerCamera(std::move(camera));

	/* Enable hot-unplug notifications. */
	hotplugMediaDevice(media);

	return true;
}

int UVCCameraData::initMetadata(MediaDevice *media)
{
	const std::vector<MediaEntity *> &entities = media->entities();

	/*
	 * \todo The kernel doesn't expose the relationship between image
	 * capture video devices and metadata capture video devices. Until this
	 * gets fixed on the kernel side, do our best by picking one metadata
	 * capture video device.
	 */
	auto metadata = std::find_if(entities.begin(), entities.end(),
				     [](MediaEntity *e) {
					     return e->type() == MediaEntity::Type::V4L2VideoDevice
                                                       && (e->pads().size() == 0);
				     });

	if (metadata == entities.end()){
		metadata_ = nullptr;
		return -ENODEV;
	}

	/* configure the metadata node */
	metadata_ = std::make_unique<V4L2VideoDevice>(*metadata);

	int ret = metadata_->open();
	if (ret) {
		metadata_ = nullptr;
		return ret;
	}

	if (!metadata_->caps().isMeta()) {
		/*
		 * UVC Devices are usually only anticipated to expose two
		 * devices, so we assume the non-default device is the metadata
		 * device node
		 */
		metadata_ = nullptr;
		return -EINVAL;
	}
	return 0;
}

int UVCCameraData::init(MediaDevice *media)
{
	int ret;

	/* Locate and initialise the camera data with the default video node. */
	const std::vector<MediaEntity *> &entities = media->entities();
	auto entity = std::find_if(entities.begin(), entities.end(),
				   [](MediaEntity *e) {
					   return e->flags() & MEDIA_ENT_FL_DEFAULT;
				   });
	if (entity == entities.end()) {
		LOG(UVC, Error) << "Could not find a default video device";
		return -ENODEV;
	}

	/* Create and open the video device. */
	video_ = std::make_unique<V4L2VideoDevice>(*entity);
	ret = video_->open();
	if (ret)
		return ret;

	video_->bufferReady.connect(this, &UVCCameraData::bufferReady);

	/* Generate the camera ID. */
	if (!generateId()) {
		LOG(UVC, Error) << "Failed to generate camera ID";
		return -EINVAL;
	}

	/*
	 * Populate the map of supported formats, and infer the camera sensor
	 * resolution from the largest size it advertises.
	 */
	Size resolution;
	for (const auto &format : video_->formats()) {
		PixelFormat pixelFormat = format.first.toPixelFormat();
		if (!pixelFormat.isValid())
			continue;

		formats_[pixelFormat] = format.second;

		const std::vector<SizeRange> &sizeRanges = format.second;
		for (const SizeRange &sizeRange : sizeRanges) {
			if (sizeRange.max > resolution)
				resolution = sizeRange.max;
		}
	}

	if (formats_.empty()) {
		LOG(UVC, Error)
			<< "Camera " << id_ << " (" << media->model()
			<< ") doesn't expose any supported format";
		return -EINVAL;
	}

	/* Populate the camera properties. */
	properties_.set(properties::Model, utils::toAscii(media->model()));

	/*
	 * Derive the location from the device removable attribute in sysfs.
	 * Non-removable devices are assumed to be front as we lack detailed
	 * location information, and removable device are considered external.
	 *
	 * The sysfs removable attribute is derived from the ACPI _UPC attribute
	 * if available, or from the USB hub descriptors otherwise. ACPI data
	 * may not be very reliable, and the USB hub descriptors may not be
	 * accurate on DT-based platforms. A heuristic may need to be
	 * implemented later if too many devices end up being miscategorized.
	 *
	 * \todo Find a way to tell front and back devices apart. This could
	 * come from the ACPI _PLD, but that may be even more unreliable than
	 * the _UPC.
	 */
	properties::LocationEnum location = properties::CameraLocationExternal;
	std::ifstream file(video_->devicePath() + "/../removable");
	if (file.is_open()) {
		std::string value;
		std::getline(file, value);
		file.close();

		if (value == "fixed")
			location = properties::CameraLocationFront;
	}

	properties_.set(properties::Location, location);

	properties_.set(properties::PixelArraySize, resolution);
	properties_.set(properties::PixelArrayActiveAreas, { Rectangle(resolution) });

	/* Initialise the supported controls. */
	ControlInfoMap::Map ctrls;

	for (const auto &ctrl : video_->controls()) {
		uint32_t cid = ctrl.first->id();
		const ControlInfo &info = ctrl.second;

		addControl(cid, info, &ctrls);
	}

	controlInfo_ = ControlInfoMap(std::move(ctrls), controls::controls);

	ret = initMetadata(media);
	if (ret) {
		LOG(UVC, Error) << "Could not find a metadata video device.";
		return 0;
	}

	metadata_->bufferReady.connect(this, &UVCCameraData::bufferReadyMetadata);

	return 0;
}

bool UVCCameraData::generateId()
{
	const std::string path = video_->devicePath();

	/* Create a controller ID from first device described in firmware. */
	std::string controllerId;
	std::string searchPath = path;
	while (true) {
		std::string::size_type pos = searchPath.rfind('/');
		if (pos <= 1) {
			LOG(UVC, Error) << "Can not find controller ID";
			return false;
		}

		searchPath = searchPath.substr(0, pos);

		controllerId = sysfs::firmwareNodePath(searchPath);
		if (!controllerId.empty())
			break;
	}

	/*
	 * Create a USB ID from the device path which has the known format:
	 *
	 *	path = bus, "-", ports, ":", config, ".", interface ;
	 *	bus = number ;
	 *	ports = port, [ ".", ports ] ;
	 *	port = number ;
	 *	config = number ;
	 *	interface = number ;
	 *
	 * Example: 3-2.4:1.0
	 *
	 * The bus is not guaranteed to be stable and needs to be stripped from
	 * the USB ID. The final USB ID is built up of the ports, config and
	 * interface properties.
	 *
	 * Example 2.4:1.0.
	 */
	std::string usbId = utils::basename(path.c_str());
	usbId = usbId.substr(usbId.find('-') + 1);

	/* Creata a device ID from the USB devices vendor and product ID. */
	std::string deviceId;
	for (const char *name : { "idVendor", "idProduct" }) {
		std::ifstream file(path + "/../" + name);

		if (!file.is_open())
			return false;

		std::string value;
		std::getline(file, value);
		file.close();

		if (!deviceId.empty())
			deviceId += ":";

		deviceId += value;
	}

	id_ = controllerId + "-" + usbId + "-" + deviceId;
	return true;
}

void UVCCameraData::addControl(uint32_t cid, const ControlInfo &v4l2Info,
			       ControlInfoMap::Map *ctrls)
{
	const ControlId *id;
	ControlInfo info;

	/* Map the control ID. */
	switch (cid) {
	case V4L2_CID_BRIGHTNESS:
		id = &controls::Brightness;
		break;
	case V4L2_CID_CONTRAST:
		id = &controls::Contrast;
		break;
	case V4L2_CID_SATURATION:
		id = &controls::Saturation;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		id = &controls::AeEnable;
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		id = &controls::ExposureTime;
		break;
	case V4L2_CID_GAIN:
		id = &controls::AnalogueGain;
		break;
	default:
		return;
	}

	/* Map the control info. */
	int32_t min = v4l2Info.min().get<int32_t>();
	int32_t max = v4l2Info.max().get<int32_t>();
	int32_t def = v4l2Info.def().get<int32_t>();

	switch (cid) {
	case V4L2_CID_BRIGHTNESS: {
		/*
		 * The Brightness control is a float, with 0.0 mapped to the
		 * default value. The control range is [-1.0, 1.0], but the V4L2
		 * default may not be in the middle of the V4L2 range.
		 * Accommodate this by restricting the range of the libcamera
		 * control, but always within the maximum limits.
		 */
		float scale = std::max(max - def, def - min);

		info = ControlInfo{
			{ static_cast<float>(min - def) / scale },
			{ static_cast<float>(max - def) / scale },
			{ 0.0f }
		};
		break;
	}

	case V4L2_CID_SATURATION:
		/*
		 * The Saturation control is a float, with 0.0 mapped to the
		 * minimum value (corresponding to a fully desaturated image)
		 * and 1.0 mapped to the default value. Calculate the maximum
		 * value accordingly.
		 */
		info = ControlInfo{
			{ 0.0f },
			{ static_cast<float>(max - min) / (def - min) },
			{ 1.0f }
		};
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		info = ControlInfo{ false, true, true };
		break;

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		/*
		 * ExposureTime is in units of 1 µs, and UVC expects
		 * V4L2_CID_EXPOSURE_ABSOLUTE in units of 100 µs.
		 */
		info = ControlInfo{
			{ min * 100 },
			{ max * 100 },
			{ def * 100 }
		};
		break;

	case V4L2_CID_CONTRAST:
	case V4L2_CID_GAIN: {
		/*
		 * The Contrast and AnalogueGain controls are floats, with 1.0
		 * mapped to the default value. UVC doesn't specify units, and
		 * cameras have been seen to expose very different ranges for
		 * the controls. Arbitrarily assume that the minimum and
		 * maximum values are respectively no lower than 0.5 and no
		 * higher than 4.0.
		 */
		float m = (4.0f - 1.0f) / (max - def);
		float p = 1.0f - m * def;

		if (m * min + p < 0.5f) {
			m = (1.0f - 0.5f) / (def - min);
			p = 1.0f - m * def;
		}

		info = ControlInfo{
			{ m * min + p },
			{ m * max + p },
			{ 1.0f }
		};
		break;
	}

	default:
		info = v4l2Info;
		break;
	}

	ctrls->emplace(id, info);
}

void UVCCameraData::completeRequest(FrameBuffer *buffer, uint64_t timestamp)
{
	Request *request = buffer->request();
	request->metadata().set(controls::SensorTimestamp, timestamp);

	pipe()->completeBuffer(request, buffer);
	pipe()->completeRequest(request);
}

/**
 * \brief The metadata data stream stopped while we're still getting video
 * buffers.  Complete the unfinished video buffers and requests.
 */
void UVCCameraData::handleUnfinishedRequests()
{
	while (!pendingVideoBuffers_.empty()) {
		FrameBuffer *oldBuffer = pendingVideoBuffers_.front();

		completeRequest(oldBuffer, oldBuffer->metadata().timestamp);
		pendingVideoBuffers_.pop();
	}
}

/* Close the metadata node so we don't get inaccurate timestamps*/
void UVCCameraData::endCorruptedMetadataStream()
{	LOG(UVC, Error)
		<< "UVC metadata stream corrupted. Reverting to driver timestamps.";
	metadata_->streamOff();

	releaseMetadataBuffers();
	
	metadata_ = nullptr;
	
	handleUnfinishedRequests();
}


/**
 * \brief Calculate a more accurate host SOF
 * \param[in] sofHost The usb clock time taken by the host
 * \param[in] sofDevice The usb clock time taken by the device
 * that has full 11 bit precision.
 *
 * The precision of the host sof may be lower than the expected 11 bits.
 * Obtain a more precise host sof by adding back in the lower 8 bits
 * of the difference between the host sof and the more precise device SOF.
 *
 * \return An updated usb clock time for the host with 11 bits of precision
 */
static unsigned short recoverHostSOF(unsigned short sofHost, unsigned short sofDevice)
{
	char sofDelta;

	sofDelta = (sofHost - sofDevice) & 255;

	return (sofDevice + sofDelta) & 2047;
}

/**
 * \brief Convert the presentation timestamp recorded by the UVC device to
 * a host timestamp.
 * \param[in] p1 Timestamps taken by the usb clock, the device, and the host
 * at an early point in time and provided by the UVC driver as metadata
 * \param[in] p2 Timestamps taken by the usb clock, the device, and the host
 * taken shortly after the presentation timestamp and provided by the UVC driver
 * as metadata
 * \param[in] PTS The presentation time stamp in device time to convert
 * to a host timestamp.
 *
 * The following device to system clock timestamp conversion algorithm is based
 * on the Linux kernel's implementation of UVC timestamp calculation.
 *
 * To convert the presentation time stamp provided by the device to a system
 * clock timestamp, first convert the pts to the usb frame number (sof),
 * and then from the usb sof to the system timestamp.  The relationship between
 * the device clock and the usb clock is linear, as is the relationship between
 * the usb clock and the system clock.  To calculate the equations needed for the
 * conversion, V4L2 provides a metadata packet with a source timestamp (stc)
 * and a usb clock sof taken at the same point in time, as well as a system
 * timestamp and a usb clock sof pairing.
 *
 * Two sets of the timestamp metadata are used to recover this linear relationship
 * and convert the pts into system clock time.
 *
 * \return The PTS timestamp converted to system clock time.
 */
unsigned long long calculateTimestamp(const UVCTimestampData &p1,
				      const UVCTimestampData &p2,
				      const unsigned int PTS)
{
	unsigned short sof1Device;
	unsigned short sof2Device;
	unsigned int stc1;
	unsigned int stc2;
	unsigned short sof1Host;
	unsigned short sof2Host;
	unsigned int mean;
	unsigned int pts;
	unsigned long long hostTS1;
	unsigned long long hostTS2;

	float sof;
	pts = PTS;

	/* Add 2048 to both sof values to prevent underflow */
	sof1Device = p1.sofDevice + 2048;
	stc1 = p1.stcDevice;
	sof2Device = p2.sofDevice + 2048;
	stc2 = p2.stcDevice;

	/* Subtract out the first point's host timestamp for simplicity */
	hostTS1 = 0;
	hostTS2 = p2.tsHost - p1.tsHost;

	/*
	 * Step 1: Convert the pts into an sof usb clock time
	 * from p1 and p2's stc and sof time pairs.
	 *
	 * The equation is:
	 * sof = ((sof1 - sof2) *pts + sof1 * stc2 - sof2 * stc1) / ( stc2 - stc1)
	 */

	/* If the sof field rolled over, add 2048 so we can extrapolate the line. */
	if (sof2Device < sof1Device) {
		sof2Device += 2048;
	}

	/*
	 * If the stc value rolled over, subtract half the 32 bit range from the
	 * stc and pts values so they fit in the rollover window.
	 */
	if (stc2 < stc1) {
		stc1 -= (1 << 31);
		stc2 -= (1 << 31);
		pts -= (1 << 31);
	}

	/* Cast the values or they may overflow at the multiplication step */
	sof = static_cast<float>(
		      (static_cast<unsigned long long>((sof2Device - sof1Device)) * static_cast<unsigned long long>(pts)
			   + static_cast<unsigned long long>(sof1Device) * static_cast<unsigned long long>(stc2) 
			   - static_cast<unsigned long long>(sof2Device) * static_cast<unsigned long long>(stc1))) /
	      static_cast<float>((stc2 - stc1));

	/*
	 * Step 2: Similar to Step1, convert the calculated sof
	 * to system timestamp from p1 and p2's host timestamp and usb sof pairs
	 *
	 * The equation is:
	 * timestamp = ((ts2 - ts1) * sof + ts1 * sof2 - ts2 * sof1) / (sof2 - sof1)
	 */
	sof1Host = recoverHostSOF(p1.sofHost, p1.sofDevice) + 2048;
	sof2Host = recoverHostSOF(p2.sofHost, p2.sofDevice) + 2048;

	if (sof2Host < sof1Host) {
		sof2Host += 2048;
	}

	/*
	 * This accounts for the possibility that the calculated sof
	 * rolled over and the host sof data fields did not.
	 */
	mean = (sof1Host + sof2Host) / 2;

	if ((mean - 1024) > sof) {
		sof += 2048;
	} else if (sof > mean + 1024) {
		sof -= 2048;
	}

	/* note: hostTS 1 has been set to zero so the difference is just the value of hostTS2.*/
	unsigned long long result = static_cast<unsigned long long>(
		(hostTS2 * sof + hostTS1 * sof2Host - hostTS2 * sof1Host) / (sof2Host - sof1Host));

	/* Add the subtracted system timestamp from p1 back in */
	return result + p1.tsHost;
}

/*
 * If the front of the metadata buffer and video buffer queues do not 
 * have matching sequence numbers, sort through these queues to see
 * if sequence numbers arrived out of order.  At the same time, if any 
 * video buffers are unmatched with metadata, use the driver timestamp.
 */
UVCCameraData::MatchResult UVCCameraData::FindAndCompleteBuffer() 
{
	if (pendingMetadata_.empty() || pendingVideoBuffers_.empty()){
		return MatchNotFound;
	}

	unsigned int mdSeq = std::get<0>(pendingMetadata_.front()) + frameStart_;
	unsigned int vdSeq = pendingVideoBuffers_.front()->metadata().sequence;

	if (mdSeq==vdSeq){
		unsigned long long timestamp = pendingVideoBuffers_.front()->metadata().timestamp;
		if (timeSamples_.size() > 1) {
				timestamp = calculateTimestamp(std::get<1>(pendingMetadata_.front()),
							       timeSamples_.back(),
							       timeSamples_.back().ptsDevice);
		}
		completeRequest(pendingVideoBuffers_.front(), timestamp);
		pendingVideoBuffers_.pop();
		pendingMetadata_.pop();
		return MatchFound;
	}
	//\todo: handle if they don't match up: search for the correct one!

	return MatchNotFound;
}


/*
 * If there is a metadata buffer that hasn't been matched with a
 * video buffer, check to see if it matches this video buffer.
 *
 * If there is a match, use the timestamp stored in the metadata queue
 * for this video buffer's request. Complete this video buffer
 * and its request.
 *
 * If there are no metadata buffers available to check for a match,
 * push this video buffer's request object to the queue. It may
 * be that the metadata buffer has not yet arrived.
 * When the matching metadata buffer does come in, it will handle
 * completion of the buffer and request.
 *
 * If more than maxVidBuffersInQueue_ video buffers have been added
 * to the queue, something is wrong with the metadata stream and
 * we can no longer use UVC metadata packets for timestamps.
 * Complete all of the outstanding requests and turn off metadata
 * stream use.
 */
void UVCCameraData::bufferReady(FrameBuffer *buffer)
{
	if (!metadata_ || buffer->metadata().sequence < frameStart_) {
		completeRequest(buffer, buffer->metadata().timestamp);
		return;
	}

	pendingVideoBuffers_.push(buffer);

	FindAndCompleteBuffer();

	/*
	 * Deal with video buffers that haven't been completed, and with
	 * buffers whose metadata information arrived out of order.
	 */
	if (pendingVideoBuffers_.size() > maxVidBuffersInQueue_ 
		&& (buffer->metadata().status != FrameMetadata::Status::FrameCancelled)) {
		endCorruptedMetadataStream();
	}
}

void UVCCameraData::addTimestampData(uvc_meta_buf &rawMetadata, UVCTimingBuf &packed)
{
	/*
	 * Copy over the buffer packet from the raw Metadata
	 * into values we can use. Populate the storage struct
	 * with the data we need to calculate timestamps.
	 * Add to the circular queue.
	 */
	UVCTimestampData data;
	data.ptsDevice = packed.pts;
	data.sofDevice = packed.sofDevice;
	data.stcDevice = packed.stc;
	data.sofHost = rawMetadata.sof;
	data.tsHost = rawMetadata.ns;

	if (timeSamples_.size() == bufferRingSize_) {
		timeSamples_.pop_front();
	}

	timeSamples_.push_back(data);
}

void UVCCameraData::bufferReadyMetadata(FrameBuffer *buffer)
{
	if (!metadata_ || 
			buffer->metadata().status != FrameMetadata::Status::FrameSuccess) {
		return;
	}

	/*
	 * The metadata stream seems to start at seq 1 and libcamera
	 * sets the start sequence to 0, so it's necessary to add one
	 * to match this buffer with the correct video frame buffer.
	 *
	 * \todo: Is there a better way to do this?  What is the root cause?
	 */
	int pos = buffer->cookie();

	Span<uint8_t> planeData(mappedMetaAddresses_[pos].get(),
				sizeof(uvc_meta_buf) + sizeof(UVCTimingBuf));
	uvc_meta_buf *metaBufPlane = reinterpret_cast<uvc_meta_buf *>(planeData.data());
	UVCTimingBuf *timeBufPlane = reinterpret_cast<UVCTimingBuf *>(&planeData.data()[sizeof(uvc_meta_buf)]);

	size_t UVCPayloadHeaderSize = sizeof(metaBufPlane->length) +
				      sizeof(metaBufPlane->flags) + sizeof(UVCTimingBuf);
	if (metaBufPlane->length < UVCPayloadHeaderSize) {
		endCorruptedMetadataStream();
		return;
	}

	addTimestampData(*metaBufPlane, *timeBufPlane);
	pendingMetadata_.push(
			std::make_tuple(buffer->metadata().sequence, timeSamples_.back()));

	FindAndCompleteBuffer();


	metadata_->queueBuffer(buffer);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC)

} /* namespace libcamera */
