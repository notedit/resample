package resample

/*
#cgo LDFLAGS: -lavformat -lavutil -lavcodec -lswresample
#cgo CFLAGS: -Wno-deprecated
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libswresample/swresample.h>
#include <libavutil/opt.h>
#include <string.h>

typedef struct {
	AVCodec *codec;
	AVCodecContext *codecCtx;
	AVFrame *frame;
	AVDictionary *options;
	int profile;
} FFCtx;


static inline int avcodec_profile_name_to_int(AVCodec *codec, const char *name) {
	const AVProfile *p;
	for (p = codec->profiles; p != NULL && p->profile != FF_PROFILE_UNKNOWN; p++)
		if (!strcasecmp(p->name, name))
			return p->profile;
	return FF_PROFILE_UNKNOWN;
}

int wrap_avcodec_decode_audio4(AVCodecContext *ctx, AVFrame *frame, void *data, int size, int *got) {
	struct AVPacket pkt = {.data = data, .size = size};
	return avcodec_decode_audio4(ctx, frame, got, &pkt);
}
int wrap_swresample_convert(SwrContext *avr, int *out, int outcount, int *in,  int incount) {
	return swr_convert(avr, (void *)out, outcount, (void *)in, incount);
}

void ffinit() {
	av_register_all();
}
*/
import "C"

import (
	"fmt"
	"runtime"
	"time"
	"unsafe"
)

func init() {
	C.ffinit()
}

type ffctx struct {
	ff C.FFCtx
}

// Audio sample format.
type SampleFormat uint8

const (
	U8   = SampleFormat(iota + 1) // 8-bit unsigned integer
	S16                           // signed 16-bit integer
	S32                           // signed 32-bit integer
	FLT                           // 32-bit float
	DBL                           // 64-bit float
	U8P                           // 8-bit unsigned integer in planar
	S16P                          // signed 16-bit integer in planar
	S32P                          // signed 32-bit integer in planar
	FLTP                          // 32-bit float in planar
	DBLP                          // 64-bit float in planar
	U32                           // unsigned 32-bit integer
)

func (self SampleFormat) BytesPerSample() int {
	switch self {
	case U8, U8P:
		return 1
	case S16, S16P:
		return 2
	case FLT, FLTP, S32, S32P, U32:
		return 4
	case DBL, DBLP:
		return 8
	default:
		return 0
	}
}

func (self SampleFormat) String() string {
	switch self {
	case U8:
		return "U8"
	case S16:
		return "S16"
	case S32:
		return "S32"
	case FLT:
		return "FLT"
	case DBL:
		return "DBL"
	case U8P:
		return "U8P"
	case S16P:
		return "S16P"
	case FLTP:
		return "FLTP"
	case DBLP:
		return "DBLP"
	case U32:
		return "U32"
	default:
		return "?"
	}
}

// Check if this sample format is in planar.
func (self SampleFormat) IsPlanar() bool {
	switch self {
	case S16P, S32P, FLTP, DBLP:
		return true
	default:
		return false
	}
}

type AudioFrame struct {
	SampleFormat SampleFormat // audio sample format, e.g: S16,FLTP,...
	Channels     int          // audio channel layout, e.g: CH_MONO,CH_STEREO,...
	SampleCount  int          // sample count in this frame
	SampleRate   int          // sample rate
	Data         [][]byte     // data array for planar format len(Data) > 1
}

// Split sample audio sample from this frame.
func (self AudioFrame) Slice(start int, end int) (out AudioFrame) {
	if start > end {
		panic(fmt.Sprintf("av: AudioFrame split failed start=%d end=%d invalid", start, end))
	}
	out = self
	out.Data = append([][]byte(nil), out.Data...)
	out.SampleCount = end - start
	size := self.SampleFormat.BytesPerSample() * self.Channels
	for i := range out.Data {
		out.Data[i] = out.Data[i][start*size : end*size]
	}
	return
}

// Concat two audio frames.
func (self AudioFrame) Concat(in AudioFrame) (out AudioFrame) {
	out = self
	out.Data = append([][]byte(nil), out.Data...)
	out.SampleCount += in.SampleCount
	for i := range out.Data {
		out.Data[i] = append(out.Data[i], in.Data[i]...)
	}
	return
}

func HasEncoder(name string) bool {
	return C.avcodec_find_encoder_by_name(C.CString(name)) != nil
}

func HasDecoder(name string) bool {
	return C.avcodec_find_decoder_by_name(C.CString(name)) != nil
}

type Resampler struct {
	inSampleFormat  SampleFormat
	OutSampleFormat SampleFormat
	inChannels      int
	OutChannels     int
	inSampleRate    int
	OutSampleRate   int
	avr             *C.SwrContext
}

func (self *Resampler) Resample(in AudioFrame) (out AudioFrame, err error) {
	formatChange := in.SampleRate != self.inSampleRate || in.SampleFormat != self.inSampleFormat || in.Channels != self.inChannels

	var flush AudioFrame

	if formatChange {

		self.inSampleFormat = in.SampleFormat
		self.inSampleRate = in.SampleRate
		self.inChannels = in.Channels
		avr := C.swr_alloc()
		C.av_opt_set_int(unsafe.Pointer(avr), C.CString("in_channel_layout"), C.int64_t(channel2ChannelLayout(self.inChannels)), 0)
		C.av_opt_set_int(unsafe.Pointer(avr), C.CString("out_channel_layout"), C.int64_t(channel2ChannelLayout(self.OutChannels)), 0)
		C.av_opt_set_int(unsafe.Pointer(avr), C.CString("in_sample_rate"), C.int64_t(self.inSampleRate), 0)
		C.av_opt_set_int(unsafe.Pointer(avr), C.CString("out_sample_rate"), C.int64_t(self.OutSampleRate), 0)
		C.av_opt_set_int(unsafe.Pointer(avr), C.CString("in_sample_fmt"), C.int64_t(sampleFormatAV2FF(self.inSampleFormat)), 0)
		C.av_opt_set_int(unsafe.Pointer(avr), C.CString("out_sample_fmt"), C.int64_t(sampleFormatAV2FF(self.OutSampleFormat)), 0)
		C.swr_init(avr)
		self.avr = avr
	}

	var inChannels int
	inSampleCount := in.SampleCount
	if !self.inSampleFormat.IsPlanar() {
		inChannels = 1
	} else {
		inChannels = self.inChannels
	}

	inData := make([]*C.uint8_t, inChannels)
	for i := 0; i < inChannels; i++ {
		inData[i] = (*C.uint8_t)(unsafe.Pointer(&in.Data[i][0]))
	}

	var outChannels, outLinesize, outBytesPerSample int
	outSampleCount := int(C.swr_get_out_samples(self.avr, C.int(in.SampleCount)))
	if !self.OutSampleFormat.IsPlanar() {
		outChannels = 1
		outBytesPerSample = self.OutSampleFormat.BytesPerSample() * self.OutChannels
		outLinesize = outSampleCount * outBytesPerSample
	} else {
		outChannels = self.OutChannels
		outBytesPerSample = self.OutSampleFormat.BytesPerSample()
		outLinesize = outSampleCount * outBytesPerSample
	}
	outData := make([]*C.uint8_t, outChannels)
	out.Data = make([][]byte, outChannels)
	for i := 0; i < outChannels; i++ {
		out.Data[i] = make([]byte, outLinesize)
		outData[i] = (*C.uint8_t)(unsafe.Pointer(&out.Data[i][0]))
	}
	out.Channels = self.OutChannels
	out.SampleFormat = self.OutSampleFormat
	out.SampleRate = self.OutSampleRate

	convertSamples := int(C.wrap_swresample_convert(
		self.avr,
		(*C.int)(unsafe.Pointer(&outData[0])), C.int(outSampleCount),
		(*C.int)(unsafe.Pointer(&inData[0])), C.int(inSampleCount),
	))
	if convertSamples < 0 {
		err = fmt.Errorf("ffmpeg: avresample_convert_frame failed")
		return
	}

	out.SampleCount = convertSamples
	if convertSamples < outSampleCount {
		for i := 0; i < outChannels; i++ {
			out.Data[i] = out.Data[i][:convertSamples*outBytesPerSample]
		}
	}

	if flush.SampleCount > 0 {
		out = flush.Concat(out)
	}

	return
}

func (self *Resampler) Close() {
	C.swr_free(&self.avr)
}

func newFFCtxByCodec(codec *C.AVCodec) (ff *ffctx, err error) {
	ff = &ffctx{}
	ff.ff.codec = codec
	ff.ff.codecCtx = C.avcodec_alloc_context3(codec)
	ff.ff.profile = C.FF_PROFILE_UNKNOWN
	runtime.SetFinalizer(ff, freeFFCtx)
	return
}

func freeFFCtx(self *ffctx) {
	ff := &self.ff
	if ff.frame != nil {
		C.av_frame_free(&ff.frame)
	}
	if ff.codecCtx != nil {
		C.avcodec_close(ff.codecCtx)
		C.av_free(unsafe.Pointer(ff.codecCtx))
		ff.codecCtx = nil
	}
	if ff.options != nil {
		C.av_dict_free(&ff.options)
	}
}

type AudioEncoder struct {
	ff               *ffctx
	SampleRate       int
	Bitrate          int
	Channels         int
	SampleFormat     SampleFormat
	FrameSampleCount int
	framebuf         AudioFrame
	resampler        *Resampler
}

func sampleFormatAV2FF(sampleFormat SampleFormat) (ffsamplefmt int32) {
	switch sampleFormat {
	case U8:
		ffsamplefmt = C.AV_SAMPLE_FMT_U8
	case S16:
		ffsamplefmt = C.AV_SAMPLE_FMT_S16
	case S32:
		ffsamplefmt = C.AV_SAMPLE_FMT_S32
	case FLT:
		ffsamplefmt = C.AV_SAMPLE_FMT_FLT
	case DBL:
		ffsamplefmt = C.AV_SAMPLE_FMT_DBL
	case U8P:
		ffsamplefmt = C.AV_SAMPLE_FMT_U8P
	case S16P:
		ffsamplefmt = C.AV_SAMPLE_FMT_S16P
	case S32P:
		ffsamplefmt = C.AV_SAMPLE_FMT_S32P
	case FLTP:
		ffsamplefmt = C.AV_SAMPLE_FMT_FLTP
	case DBLP:
		ffsamplefmt = C.AV_SAMPLE_FMT_DBLP
	}
	return
}

func sampleFormatFF2AV(ffsamplefmt int32) (sampleFormat SampleFormat) {
	switch ffsamplefmt {
	case C.AV_SAMPLE_FMT_U8: ///< unsigned 8 bits
		sampleFormat = U8
	case C.AV_SAMPLE_FMT_S16: ///< signed 16 bits
		sampleFormat = S16
	case C.AV_SAMPLE_FMT_S32: ///< signed 32 bits
		sampleFormat = S32
	case C.AV_SAMPLE_FMT_FLT: ///< float
		sampleFormat = FLT
	case C.AV_SAMPLE_FMT_DBL: ///< double
		sampleFormat = DBL
	case C.AV_SAMPLE_FMT_U8P: ///< unsigned 8 bits, planar
		sampleFormat = U8P
	case C.AV_SAMPLE_FMT_S16P: ///< signed 16 bits, planar
		sampleFormat = S16P
	case C.AV_SAMPLE_FMT_S32P: ///< signed 32 bits, planar
		sampleFormat = S32P
	case C.AV_SAMPLE_FMT_FLTP: ///< float, planar
		sampleFormat = FLTP
	case C.AV_SAMPLE_FMT_DBLP: ///< double, planar
		sampleFormat = DBLP
	}
	return
}

func (self *AudioEncoder) SetSampleFormat(fmt SampleFormat) (err error) {
	self.SampleFormat = fmt
	return
}

func (self *AudioEncoder) SetSampleRate(rate int) (err error) {
	self.SampleRate = rate
	return
}

func (self *AudioEncoder) SetChannels(ch int) (err error) {
	self.Channels = ch
	return
}

func (self *AudioEncoder) SetBitrate(bitrate int) (err error) {
	self.Bitrate = bitrate
	return
}

func (self *AudioEncoder) SetOption(key string, val interface{}) (err error) {
	ff := &self.ff.ff

	sval := fmt.Sprint(val)
	if key == "profile" {
		ff.profile = C.avcodec_profile_name_to_int(ff.codec, C.CString(sval))
		if ff.profile == C.FF_PROFILE_UNKNOWN {
			err = fmt.Errorf("ffmpeg: profile `%s` invalid", sval)
			return
		}
		return
	}

	C.av_dict_set(&ff.options, C.CString(key), C.CString(sval), 0)
	return
}

func (self *AudioEncoder) GetOption(key string, val interface{}) (err error) {
	ff := &self.ff.ff
	entry := C.av_dict_get(ff.options, C.CString(key), nil, 0)
	if entry == nil {
		err = fmt.Errorf("ffmpeg: GetOption failed: `%s` not exists", key)
		return
	}
	switch p := val.(type) {
	case *string:
		*p = C.GoString(entry.value)
	case *int:
		fmt.Sscanf(C.GoString(entry.value), "%d", p)
	default:
		err = fmt.Errorf("ffmpeg: GetOption failed: val must be *string or *int receiver")
		return
	}
	return
}

func (self *AudioEncoder) Setup() (err error) {
	ff := &self.ff.ff

	if self.SampleFormat == SampleFormat(0) {
		self.SampleFormat = sampleFormatFF2AV(*ff.codec.sample_fmts)
	}

	if self.SampleRate == 0 {
		self.SampleRate = 48000
	}
	if self.Channels == 0 {
		self.Channels = 2
	}

	ff.codecCtx.sample_fmt = sampleFormatAV2FF(self.SampleFormat)
	ff.codecCtx.sample_rate = C.int(self.SampleRate)
	ff.codecCtx.bit_rate = C.int64_t(self.Bitrate)
	ff.codecCtx.channel_layout = channel2ChannelLayout(self.Channels)

	if C.avcodec_open2(ff.codecCtx, ff.codec, nil) != 0 {
		err = fmt.Errorf("ffmpeg: encoder: avcodec_open2 failed")
		return
	}
	self.SampleFormat = sampleFormatFF2AV(ff.codecCtx.sample_fmt)
	self.FrameSampleCount = int(ff.codecCtx.frame_size)

	ff.frame = C.av_frame_alloc()

	return
}

func (self *AudioEncoder) prepare() (err error) {
	ff := &self.ff.ff

	if ff.frame == nil {
		if err = self.Setup(); err != nil {
			return
		}
	}

	return
}

func (self *AudioEncoder) encodeOne(frame AudioFrame) (gotpkt bool, pkt []byte, err error) {
	if err = self.prepare(); err != nil {
		return
	}

	ff := &self.ff.ff

	cpkt := C.AVPacket{}
	cgotpkt := C.int(0)
	audioFrameAssignToFF(frame, ff.frame)

	cerr := C.avcodec_encode_audio2(ff.codecCtx, &cpkt, ff.frame, &cgotpkt)
	if cerr < C.int(0) {
		err = fmt.Errorf("ffmpeg: avcodec_encode_audio2 failed: %d", cerr)
		return
	}

	if cgotpkt != 0 {
		gotpkt = true
		pkt = C.GoBytes(unsafe.Pointer(cpkt.data), cpkt.size)
		C.av_packet_unref(&cpkt)

	}

	return
}

func (self *AudioEncoder) resample(in AudioFrame) (out AudioFrame, err error) {
	if self.resampler == nil {
		self.resampler = &Resampler{
			OutSampleFormat: self.SampleFormat,
			OutSampleRate:   self.SampleRate,
			OutChannels:     self.Channels,
		}
	}
	if out, err = self.resampler.Resample(in); err != nil {
		return
	}
	return
}

func (self *AudioEncoder) Encode(frame AudioFrame) (pkts [][]byte, err error) {
	var gotpkt bool
	var pkt []byte

	if frame.SampleFormat != self.SampleFormat || frame.Channels != self.Channels || frame.SampleRate != self.SampleRate {
		if frame, err = self.resample(frame); err != nil {
			return
		}
	}

	if self.FrameSampleCount != 0 {
		if self.framebuf.SampleCount == 0 {
			self.framebuf = frame
		} else {
			self.framebuf = self.framebuf.Concat(frame)
		}
		for self.framebuf.SampleCount >= self.FrameSampleCount {
			frame := self.framebuf.Slice(0, self.FrameSampleCount)
			if gotpkt, pkt, err = self.encodeOne(frame); err != nil {
				return
			}
			if gotpkt {
				pkts = append(pkts, pkt)
			}
			self.framebuf = self.framebuf.Slice(self.FrameSampleCount, self.framebuf.SampleCount)
		}
	} else {
		if gotpkt, pkt, err = self.encodeOne(frame); err != nil {
			return
		}
		if gotpkt {
			pkts = append(pkts, pkt)
		}
	}

	return
}

func (self *AudioEncoder) PacketDuration(data []byte) (dur time.Duration, err error) {
	ff := &self.ff.ff
	duration := C.av_get_audio_frame_duration(ff.codecCtx, C.int(len(data)))
	dur = time.Duration(int(duration)) * time.Second / time.Duration(self.SampleRate)
	return
}

func (self *AudioEncoder) Close() {
	freeFFCtx(self.ff)
	if self.resampler != nil {
		self.resampler.Close()
		self.resampler = nil
	}
}

func audioFrameAssignToAVParams(f *C.AVFrame, frame *AudioFrame) {
	frame.SampleFormat = sampleFormatFF2AV(int32(f.format))
	frame.Channels = int(f.channels)
	frame.SampleRate = int(f.sample_rate)
}

func audioFrameAssignToAVData(f *C.AVFrame, frame *AudioFrame) {

	frame.SampleCount = int(f.nb_samples)

	fmt.Println("SampleCount", frame.SampleCount)

	var outChannels, outLinesize, outBytesPerSample int
	if !frame.SampleFormat.IsPlanar() {
		outChannels = 1
		outBytesPerSample = frame.SampleFormat.BytesPerSample() * frame.Channels
		outLinesize = frame.SampleCount * outBytesPerSample
	} else {
		outChannels = frame.Channels
		outBytesPerSample = frame.SampleFormat.BytesPerSample()
		outLinesize = frame.SampleCount * outBytesPerSample
	}

	frame.Data = make([][]byte, outChannels)

	if outLinesize != int(f.linesize[0]) {
		fmt.Println("linesize does not match", outLinesize, int(f.linesize[0]))
	}

	for i := 0; i < outChannels; i++ {
		frame.Data[i] = make([]byte, outLinesize)
		frame.Data[i] = C.GoBytes(unsafe.Pointer(f.data[i]), f.linesize[0])
	}
}

func audioFrameAssignToAV(f *C.AVFrame, frame *AudioFrame) {
	audioFrameAssignToAVParams(f, frame)
	audioFrameAssignToAVData(f, frame)
}

func audioFrameAssignToFFParams(frame AudioFrame, f *C.AVFrame) {
	f.format = C.int(sampleFormatAV2FF(frame.SampleFormat))
	f.channel_layout = channel2ChannelLayout(frame.Channels)
	f.sample_rate = C.int(frame.SampleRate)
	f.channels = C.int(frame.Channels)
}

func audioFrameAssignToFFData(frame AudioFrame, f *C.AVFrame) {
	f.nb_samples = C.int(frame.SampleCount)
	for i := range frame.Data {
		f.data[i] = (*C.uint8_t)(unsafe.Pointer(&frame.Data[i][0]))
		f.linesize[i] = C.int(len(frame.Data[i]))
	}
}

func audioFrameAssignToFF(frame AudioFrame, f *C.AVFrame) {
	audioFrameAssignToFFParams(frame, f)
	audioFrameAssignToFFData(frame, f)
}

type AudioDecoder struct {
	ff           *ffctx
	Channels     int
	SampleFormat SampleFormat
	SampleRate   int
	Extradata    []byte
}

func (self *AudioDecoder) SetSampleFormat(fmt SampleFormat) (err error) {
	self.SampleFormat = fmt
	return
}

func (self *AudioDecoder) SetSampleRate(rate int) (err error) {
	self.SampleRate = rate
	return
}

func (self *AudioDecoder) SetChannels(ch int) (err error) {
	self.Channels = ch
	return
}

func (self *AudioDecoder) Setup() (err error) {
	ff := &self.ff.ff

	ff.frame = C.av_frame_alloc()

	if len(self.Extradata) > 0 {
		ff.codecCtx.extradata = (*C.uint8_t)(unsafe.Pointer(&self.Extradata[0]))
		ff.codecCtx.extradata_size = C.int(len(self.Extradata))
	}

	ff.codecCtx.sample_rate = C.int(self.SampleRate)
	ff.codecCtx.channel_layout = channel2ChannelLayout(self.Channels)
	ff.codecCtx.channels = C.int(self.Channels)
	if C.avcodec_open2(ff.codecCtx, ff.codec, nil) != 0 {
		err = fmt.Errorf("ffmpeg: decoder: avcodec_open2 failed")
		return
	}
	self.SampleFormat = sampleFormatFF2AV(ff.codecCtx.sample_fmt)
	if self.SampleRate == 0 {
		self.SampleRate = int(ff.codecCtx.sample_rate)
	}

	return
}

func (self *AudioDecoder) Decode(pkt []byte) (gotframe bool, frame AudioFrame, err error) {
	ff := &self.ff.ff

	cgotframe := C.int(0)
	cerr := C.wrap_avcodec_decode_audio4(ff.codecCtx, ff.frame, unsafe.Pointer(&pkt[0]), C.int(len(pkt)), &cgotframe)
	if cerr < C.int(0) {
		err = fmt.Errorf("ffmpeg: avcodec_decode_audio4 failed: %d", cerr)
		return
	}

	if cgotframe != C.int(0) {
		gotframe = true
		audioFrameAssignToAV(ff.frame, &frame)
		frame.SampleRate = self.SampleRate

	}

	return
}

func (self *AudioDecoder) PacketDuration(data []byte) (dur time.Duration, err error) {
	ff := &self.ff.ff
	duration := C.av_get_audio_frame_duration(ff.codecCtx, C.int(len(data)))
	dur = time.Duration(int(duration)) * time.Second / time.Duration(self.SampleRate)
	return
}

func (self *AudioDecoder) Close() {
	freeFFCtx(self.ff)
}

func NewAudioEncoder(name string) (enc *AudioEncoder, err error) {
	_enc := &AudioEncoder{}

	codec := C.avcodec_find_encoder_by_name(C.CString(name))
	if codec == nil || C.avcodec_get_type(codec.id) != C.AVMEDIA_TYPE_AUDIO {
		err = fmt.Errorf("ffmpeg: cannot find audio encoder name=%s", name)
		return
	}

	if _enc.ff, err = newFFCtxByCodec(codec); err != nil {
		return
	}
	enc = _enc
	return
}

func NewAudioDecoder(name string) (dec *AudioDecoder, err error) {
	_dec := &AudioDecoder{}

	codec := C.avcodec_find_decoder_by_name(C.CString(name))
	if codec == nil || C.avcodec_get_type(codec.id) != C.AVMEDIA_TYPE_AUDIO {
		err = fmt.Errorf("ffmpeg: cannot find audio decoder name=%s", name)
		return
	}

	if _dec.ff, err = newFFCtxByCodec(codec); err != nil {
		return
	}

	dec = _dec

	return

}

func channel2ChannelLayout(channel int) (layout C.uint64_t) {
	if channel == 1 {
		return C.uint64_t(C.AV_CH_LAYOUT_MONO)
	}
	if channel == 2 {
		return C.uint64_t(C.AV_CH_LAYOUT_STEREO)
	}
	return 0
}
