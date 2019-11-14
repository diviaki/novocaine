#include "Novocaine.h"

#define kInputBus 1
#define kOutputBus 0

#ifdef DEBUG
#define OPT_LOG
#endif

#ifndef OPT_LOG
#define NSLog(...)
#endif

#ifndef OPT_LOG
#define CheckError( function, ...) function
#else
#ifdef __cplusplus
extern "C" {
#endif
	
    void CheckError(OSStatus error, const char *operation)
    {
        if (error == noErr) return;
        
        char str[20];
        // see if it appears to be a 4-char-code
        *(UInt32 *)(str + 1) = CFSwapInt32HostToBig(error);
        if (isprint(str[1]) && isprint(str[2]) && isprint(str[3]) && isprint(str[4])) {
            str[0] = str[5] = '\'';
            str[6] = '\0';
        } else
            // no, format it as an integer
            sprintf(str, "%d", (int)error);
        
        fprintf(stderr, "Error: %s (%s)\n", operation, str);
        
        exit(1);
    }
#endif
	
OSStatus renderCallback (void						* inRefCon,
						 AudioUnitRenderActionFlags	* ioActionFlags,
						 const AudioTimeStamp 		* inTimeStamp,
						 UInt32						inOutputBusNumber,
						 UInt32						inNumberFrames,
						 AudioBufferList			* ioData);

#ifdef __cplusplus
}
#endif

UInt32 numOutputChannels;
Float64 samplingRate;
NovocaineOutputBlock outputBlock;

AudioUnit audioUnit;
AudioStreamBasicDescription outputFormat;
BOOL playing;
float *outData;

#if defined (USING_OSX)
AudioFileWriter* fileWriter;
#else
#include "GameViewController.h"
#endif

void novocaine_init()
{
	
	// Initialize a float buffer to hold audio
	outData = (float *)calloc(8192, sizeof(float)); // probably more than we'll need
	outputBlock = nil;
	
	playing = NO;
	
    // Initialize and configure the audio session, and add an interuption listener
    
#if defined ( USING_IOS )
    AVAudioSession *session = [AVAudioSession sharedInstance];

	NSError *err = nil;
    if (![session setCategory:
          AVAudioSessionCategoryPlayback
                error:&err]) {
    }

    if (![session setActive:YES error:&err]){
        NSLog(@"Couldn't activate audio session: %@", err);
    }
	
    // Audio Session Setup
    [[NSNotificationCenter defaultCenter] addObserver: gwc
					     selector: @selector(rcv_ds_rc:)
						 name: AVAudioSessionRouteChangeNotification
					       object: session];

#ifdef FORCE_FIXED_SAMPLERATE
#if !TARGET_IPHONE_SIMULATOR
    //prevent runtime switching between 44.1 (headphones) and 48kHz (built-in speakers) on latest iPhone models with iOS9
    [session setPreferredSampleRate:48000. error:nil];
#endif
#endif
	
    novocaine_checkSessionProperties();
#endif
    
    // Audio Unit Setup
    AudioComponentDescription outputDescription = {0};
    outputDescription.componentType = kAudioUnitType_Output;
#if defined (USING_IOS)
    outputDescription.componentSubType = kAudioUnitSubType_RemoteIO;
#else
	outputDescription.componentSubType = kAudioUnitSubType_HALOutput;
#endif
    outputDescription.componentManufacturer = kAudioUnitManufacturer_Apple;
	
    AudioComponent outputComponent = AudioComponentFindNext(NULL, &outputDescription);
    CheckError( AudioComponentInstanceNew(outputComponent, &audioUnit), "Couldn't create the output audio unit");

	UInt32 size;
	
#if defined ( USING_OSX )    
    // Disable output on the input unit
    // (only on Mac, since on the iPhone, the input unit is also the output unit)
	UInt32 one = 1;
    UInt32 zero = 0;

	// Enable output
    CheckError( AudioUnitSetProperty(audioUnit,
                                     kAudioOutputUnitProperty_EnableIO, 
                                     kAudioUnitScope_Output, 
                                     kOutputBus, 
                                     &one, 
                                     sizeof(one)), "Couldn't enable IO on the input scope of output unit");
    
    // Disable input
    CheckError( AudioUnitSetProperty(audioUnit,
                                     kAudioOutputUnitProperty_EnableIO, 
                                     kAudioUnitScope_Input, 
                                     kInputBus, 
                                     &zero, 
                                     sizeof(UInt32)), "Couldn't disable output on the audio unit");
	
	AudioObjectPropertyAddress propertyAddress;
	propertyAddress.mSelector = kAudioHardwarePropertyDefaultOutputDevice;
	propertyAddress.mScope = kAudioObjectPropertyScopeGlobal;
	propertyAddress.mElement = kAudioObjectPropertyElementMaster;
	
	AudioDeviceID outputDeviceID;
	size = sizeof(AudioDeviceID);
	CheckError(AudioObjectGetPropertyData(kAudioObjectSystemObject, &propertyAddress, 0, NULL, &size, &outputDeviceID), "Could not get the default device");
	
	CheckError(AudioUnitSetProperty(audioUnit,
									kAudioOutputUnitProperty_CurrentDevice,
									kAudioUnitScope_Global,
									kOutputBus,
									&outputDeviceID,
									sizeof(AudioDeviceID) ), "Couldn't set the current output audio device");
#endif
	
	//set samplerate
	size = sizeof( AudioStreamBasicDescription );
	CheckError( AudioUnitGetProperty(audioUnit,
                                     kAudioUnitProperty_StreamFormat,
                                     kAudioUnitScope_Output, 
                                     kInputBus,
                                     &outputFormat,
                                     &size ), 
               "Couldn't get the hardware output stream format");
	
# if defined ( USING_OSX )
	numOutputChannels = outputFormat.mChannelsPerFrame;
	samplingRate = outputFormat.mSampleRate;
#else
	outputFormat.mSampleRate = samplingRate;

    size = sizeof(AudioStreamBasicDescription);
	CheckError(AudioUnitSetProperty(audioUnit,
									kAudioUnitProperty_StreamFormat,
									kAudioUnitScope_Output,
									kInputBus,
									&outputFormat,
									size),
			   "Couldn't set the ASBD on the audio unit (after setting its sampling rate)");
#endif
	
    // Slap a render callback on the unit
    AURenderCallbackStruct callbackStruct;
    
    callbackStruct.inputProc = renderCallback;
    CheckError( AudioUnitSetProperty(audioUnit,
                                     kAudioUnitProperty_SetRenderCallback, 
                                     kAudioUnitScope_Input,
                                     kOutputBus,
                                     &callbackStruct, 
                                     sizeof(callbackStruct)), 
               "Couldn't set the render callback on the audio unit");

    CheckError(AudioUnitInitialize(audioUnit), "Couldn't initialize the audio unit");
	
}

void novocaine_pause()
{
	if (playing) {
		CheckError(AudioOutputUnitStop(audioUnit), "Couldn't stop the audio unit");
		playing = NO;
	}
}

void novocaine_play()
{
	if (!playing) {
		CheckError(AudioOutputUnitStart(audioUnit), "Couldn't start the audio unit");
		playing = YES;
	}
}

OSStatus renderCallback (void						* inRefCon,
                         AudioUnitRenderActionFlags	* ioActionFlags,
                         const AudioTimeStamp 		* inTimeStamp,
                         UInt32						inOutputBusNumber,
                         UInt32						inNumberFrames,
                         AudioBufferList			* ioData)
{
	float zero = 0.0;
	
	for (int iBuffer=0; iBuffer < ioData->mNumberBuffers; ++iBuffer) {
		memset(ioData->mBuffers[iBuffer].mData, 0, ioData->mBuffers[iBuffer].mDataByteSize);
	}
	
	if (!playing)
		return noErr;
	if (!outputBlock)
		return noErr;
	
	// Collect data to render from the callbacks
	outputBlock(outData, inNumberFrames, numOutputChannels);
	
#if defined(USING_OSX)
	if( fileWriter )
		[fileWriter writeNewAudio:outData numFrames:inNumberFrames numChannels:numOutputChannels];
#endif
	
	// Put the rendered data into the output buffer
#if !TARGET_IPHONE_SIMULATOR
	for (int iBuffer=0; iBuffer < ioData->mNumberBuffers; ++iBuffer) {
		
		int thisNumChannels = ioData->mBuffers[iBuffer].mNumberChannels;
		
		for (int iChannel = 0; iChannel < thisNumChannels; ++iChannel) {
			
			int interleaveOffset = iChannel;
			if (iBuffer < numOutputChannels){
				interleaveOffset += iBuffer;
			}
			
			vDSP_vsadd(outData+interleaveOffset, numOutputChannels, &zero, (float *)ioData->mBuffers[iBuffer].mData, thisNumChannels, inNumberFrames);
		}
	}
#else
	//numBytesPerSample == 2: need to convert Float -> SInt16 and also scale
	 {
		 float scale = (float)INT16_MAX;
		 vDSP_vsmul(outData, 1, &scale, outData, 1, inNumberFrames*numOutputChannels);
		 
		 for (int iBuffer=0; iBuffer < ioData->mNumberBuffers; ++iBuffer) {
			 
			 int thisNumChannels = ioData->mBuffers[iBuffer].mNumberChannels;
			 
			 for (int iChannel = 0; iChannel < thisNumChannels; ++iChannel) {
				 
				 int interleaveOffset = iChannel;
				 if (iBuffer < numOutputChannels){
					 interleaveOffset += iBuffer;
				 }
				 
				 vDSP_vfix16(outData+interleaveOffset, numOutputChannels, (SInt16 *)ioData->mBuffers[iBuffer].mData+iChannel, thisNumChannels, inNumberFrames);
			 }
		 }
	 }
#endif
	 
    return noErr;
}

#if defined (USING_IOS)

void novocaine_routechange(NSNotification* n)
{
    // Determines the reason for the route change, to ensure that it is not because of a category change.
    NSInteger  reason = [[[n userInfo] objectForKey:AVAudioSessionRouteChangeReasonKey] integerValue];

    if (reason != AVAudioSessionRouteChangeReasonCategoryChange)
    {
        novocaine_checkSessionProperties();
    }
}


// To be run ONCE per session property change and once on initialization.
void novocaine_checkSessionProperties()
{
    AVAudioSession *session = [AVAudioSession sharedInstance];

    // Check the number of output channels. Changing when (un)plugging headphones
    numOutputChannels = (UInt32)[session outputNumberOfChannels];
    NSLog(@"We've got %u output channels", (unsigned int)numOutputChannels);
    
    // Get the hardware sampling rate. This is settable, but here we're only reading.
    samplingRate = [session sampleRate];
    NSLog(@"Current sampling rate: %f", samplingRate);
	
    NSLog(@"IO buffer duration is %f", [session IOBufferDuration]);
}
#endif

#if defined ( USING_OSX )
void novocaine_recordOutput(NSString* filename)
{
	if( filename ){
    NSArray *pathComponents = [NSArray arrayWithObjects:
                               [NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES) lastObject],
                               filename,
                               nil];
    NSURL *outputFileURL = [NSURL fileURLWithPathComponents:pathComponents];

    fileWriter = [[AudioFileWriter alloc]
                       initWithAudioFileURL:outputFileURL
                       samplingRate:samplingRate
                       numChannels:numOutputChannels];
    }else{
		[fileWriter stop];
		fileWriter = nil;
    }
}
#endif








