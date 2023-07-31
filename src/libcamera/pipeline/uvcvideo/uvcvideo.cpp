/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * uvcvideo.cpp - Pipeline handler for uvcvideo devices
 */

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <memory>
#include <tuple>
#include <sys/mman.h>


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
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/sysfs.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(UVC)

/* \todo Turn this into a class with convenience functions
 */
struct UVC_Block {
	/* UVCH driver-supplied information*/
	uint64_t ts;
	__u16 sof;
	/* Device-supplied UVC payload header*/
	__u8 length;
	__u8 flags;
	__u64 PTS;
	__u8 SCR[6];
};

using unique_mapped_ptr = std::unique_ptr<UVC_Block, void (*)(UVC_Block *)>;

class UVCCameraData : public Camera::Private
{
public:
	UVCCameraData(PipelineHandler *pipe)
		: Camera::Private(pipe)
	{
	}

	int initMetadata(MediaDevice *media);
	int init(MediaDevice *media);
	void addControl(uint32_t cid, const ControlInfo &v4l2info,
			ControlInfoMap::Map *ctrls);
	void bufferReady(FrameBuffer *buffer);
	void bufferReadyMetadata(FrameBuffer *buffer);

	const std::string &id() const { return id_; }

	std::unique_ptr<V4L2VideoDevice> video_;
	std::unique_ptr<V4L2VideoDevice> metadata_;
	Stream stream_;
	std::vector<std::unique_ptr<FrameBuffer>> metadataBuffers_;
	std::vector<unique_mapped_ptr> mappedMetaAddresses_;
	bool useMetadataStream;

	std::map<PixelFormat, std::vector<SizeRange>> formats_;
	bool isStopped;
	std::queue<std::pair<Request *, FrameBuffer *>> waitingForVideoBuffer_;
	std::queue<std::pair<unsigned int, uint64_t>> waitingForMDBuffer_; //first is sequence, second is ts
private:
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
	int cleanupMetadataBuffers(Camera *camera);
	int cleanup(Camera *camera);
	Timer watchdog_; //end the request if we aren't getting metadata buffers
	utils::Duration watchdogDuration_;

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

//-Wno-error
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

int PipelineHandlerUVC::cleanupMetadataBuffers(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);
	int ret = data->metadata_->releaseBuffers();
	data->metadataBuffers_.clear(); //call the destructor for the frame buffers
	data->mappedMetaAddresses_.clear(); //unmap the buffers
	data->useMetadataStream = false;
	return ret;
}

int PipelineHandlerUVC::cleanup(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);
	cleanupMetadataBuffers(camera);
	data->video_->releaseBuffers();
	return 0;
}

/*
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
 * \retval -ENOMEM if mmap failed.
 */
int PipelineHandlerUVC::createMetadataBuffers(Camera *camera, unsigned int count)
{
	UVCCameraData *data = cameraData(camera);
	int ret = data->metadata_->allocateBuffers(count, &data->metadataBuffers_);
	if (ret != (int)count)
		return -EINVAL;
	for (unsigned int i = 0; i < count; i++) {
		std::unique_ptr<FrameBuffer> &buffer = data->metadataBuffers_[i];
		void *address = mmap(NULL, buffer->planes()[0].length,
				     PROT_READ | PROT_WRITE,
				     MAP_SHARED,
				     data->metadata_->fd(), buffer->planes()[0].offset);

		if (address == MAP_FAILED) {
			LOG(UVC, Error) << "Failed to mmap UVC metadata plane: -"
					<< strerror(errno);
			cleanupMetadataBuffers(camera);
			return -ENOMEM;
		}

		//\todo: handle multiple calls to this function better
		data->mappedMetaAddresses_.emplace_back(
			unique_mapped_ptr(new (address) UVC_Block,
					  [](UVC_Block *p) {
						  p->~UVC_Block();
						  munmap(p, sizeof(UVC_Block));
					  }));
		buffer->setCookie(i);
	}
	return ret;
}

/* Exports frame buffers for video stream, and initializes
 * and allocates the frame buffers for the metadata stream.
 */

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

	if (data->useMetadataStream) {
		if (createMetadataBuffers(camera, count) != (int)count) {
			LOG(UVC, Error) << "Unable to allocate buffers for UVC metadata stream.";
			data->useMetadataStream = false;
		}
	}

	ret = data->video_->streamOn();
	if (ret < 0) {
		cleanup(camera);
		return ret;
	}

	if (data->useMetadataStream) {
		ret = data->metadata_->streamOn();
		for (std::unique_ptr<FrameBuffer> &buf : data->metadataBuffers_) {
			ret = data->metadata_->queueBuffer(buf.get());
			if (ret < 0)
				return ret;
		}
		if (ret < 0) {
			cleanupMetadataBuffers(camera);
			return ret;
		}
	}
	return 0;
}

void PipelineHandlerUVC::stopDevice(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);

	data->video_->streamOff();

	data->isStopped = true;
	if (data->useMetadataStream) {
		data->metadata_->streamOff();
	}
	cleanup(camera);
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
	int ret;

	const std::vector<MediaEntity *> &entities = media->entities();

	std::string dev_node_name = video_->deviceNode();
	auto metadata = std::find_if(entities.begin(), entities.end(),
				     [&dev_node_name](MediaEntity *e) {
					     return e->type() == MediaEntity::Type::V4L2VideoDevice && !(e->flags() & MEDIA_ENT_FL_DEFAULT);
				     });

	if (metadata == entities.end()) {
		LOG(UVC, Error) << "Could not find a metadata video device.";
		return -ENODEV;
	}

	/* configure the metadata node */
	metadata_ = std::make_unique<V4L2VideoDevice>(*metadata);
	ret = metadata_->open();

	if (ret)
		return ret;

	if (!(metadata_->caps().isMeta())) {
		/* if the caps do not have the metadata attribute
		 * (shouldn't happen) */
		metadata_ = NULL;
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
	LOG(UVC, Gab) << "size of init queue: " << waitingForMDBuffer_.size();
    
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
	if (!ret) {
		metadata_->bufferReady.connect(this, &UVCCameraData::bufferReadyMetadata);
		useMetadataStream = true;
	} else {
		useMetadataStream = false;
	}

	/* \todo: handle a failure of the metadata opening, including arranging for the old
	 * timestamping method to be used and for appropriate user warnings. */

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

void UVCCameraData::bufferReady(FrameBuffer *buffer)
{
	LOG(UVC,Gab) <<"*****VID BUFFER READY: seq " << buffer->metadata().sequence;

	Request *request = buffer->request();

	if (!waitingForMDBuffer_.empty() && (std::get<0>(waitingForMDBuffer_.front())+1) ==buffer->metadata().sequence){
		LOG(UVC,Gab) <<"vid matched with md: seq: " << buffer->metadata().sequence;
		request->metadata().set(controls::SensorTimestamp,
		std::get<1>(waitingForMDBuffer_.front()));
		LOG(UVC, Gab) << "**ts: " << std::get<1>(waitingForMDBuffer_.front())<< " vs " << buffer->metadata().timestamp;
		pipe()->completeBuffer(request, buffer);
		pipe()->completeRequest(request);
		waitingForMDBuffer_.pop();
		return;
	}else{
		//waitingForMDBuffer_.pop();
		//waitingForMDBuffer_.pop();
		waitingForVideoBuffer_.push(std::make_pair(request, buffer));
	}

	/* \todo Use the UVC metadata to calculate a more precise timestamp */
	request->metadata().set(controls::SensorTimestamp,
				buffer->metadata().timestamp);

	if (buffer->metadata().sequence == 0){
		LOG(UVC, Gab) << "ts: " << buffer->metadata().timestamp;

		pipe()->completeBuffer(request, buffer);
		pipe()->completeRequest(request);
		waitingForVideoBuffer_.pop();
	}

	if (waitingForVideoBuffer_.size() > 3){
		LOG(UVC, Gab) << "Error: Metadata buffers are not sending. Completing requests now.";

		while (!waitingForVideoBuffer_.empty()){
			Request *oldRequest = std::get<0>(waitingForVideoBuffer_.front());
			FrameBuffer *oldBuffer = std::get<1>(waitingForVideoBuffer_.front());
			LOG(UVC, Gab) << "**ts: " << buffer->metadata().timestamp;

			pipe()->completeBuffer(oldRequest, oldBuffer);
			pipe()->completeRequest(oldRequest);
			waitingForVideoBuffer_.pop();
		}
	}


}
int c = 0;
void UVCCameraData::bufferReadyMetadata(FrameBuffer *buffer)
{

	/* The metadata stream always starts at seq 1 and libcamera sets the start sequence to 0,
	 * so it's necessary to add one to match this buffer with the correct
	 * video frame buffer. 
	 */
	unsigned int seqNumber = buffer->metadata().sequence+1;
	int pos;
	if (!useMetadataStream || buffer->metadata().status != FrameMetadata::Status::FrameSuccess){

		return;
	}	
	LOG(UVC,Gab) <<"*****META BUFFER READY: seq " << buffer->metadata().sequence ;

	/* \todo: Use the data in the metadata buffer to get a more
	* accurate timestamp. For now, just print the buffer data's timestamp
	* which was given by the driver, and also the timestamp given
	* as part of this buffer's metadata.
	*/
	pos = buffer->cookie();
	//LOG(UVC, Gab) << "md queue size: " << waitingForMDBuffer_.size();
	//LOG(UVC,Gab) <<"md sequence just pushed: " << buffer->metadata().sequence << "; queue top is: " << std::get<0>(waitingForMDBuffer_.front());

	//LOG(UVC, Gab) << "UVC Extended field's data content (our) timestamp: "
	//	      << mappedMetaAddresses_[pos]->ts;
	//LOG(UVC, Gab) << "UVC Extended field's V4L2 timestamp: "
	//	      << buffer->metadata().timestamp;
	//LOG(UVC, Gab) << "UVC Extended field's PTS: "
	//	      << mappedMetaAddresses_[pos]->PTS;

	//LOG(UVC, Gab) << utils::abs_diff(buffer->metadata().timestamp, mappedMetaAddresses_[pos]->ts);


	if (!waitingForVideoBuffer_.empty() && std::get<1>(waitingForVideoBuffer_.front())->metadata().sequence== seqNumber){
		LOG(UVC, Gab) << "md matched with vid.";
		Request *request = std::get<0>(waitingForVideoBuffer_.front());
		FrameBuffer *vidBuffer = std::get<1>(waitingForVideoBuffer_.front());
		request->metadata().set(controls::SensorTimestamp,
			mappedMetaAddresses_[pos]->ts);
		LOG(UVC, Gab) << "**ts: " << mappedMetaAddresses_[pos]->ts << " vs " << vidBuffer->metadata().timestamp;

		pipe()->completeBuffer(request, vidBuffer);
		pipe()->completeRequest(request);
		waitingForVideoBuffer_.pop();
		return;
	}

	waitingForMDBuffer_.push(std::make_pair(buffer->metadata().sequence, mappedMetaAddresses_[pos]->ts));
	LOG(UVC, Gab) << "pushing " <<  mappedMetaAddresses_[pos]->ts;

	metadata_->queueBuffer(buffer);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC)

} /* namespace libcamera */
