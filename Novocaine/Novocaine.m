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
	
    OSStatus renderCallback (void						*inRefCon,
                             AudioUnitRenderActionFlags	* ioActionFlags,
                             const AudioTimeStamp 		* inTimeStamp,
                             UInt32						inOutputBusNumber,
                             UInt32						inNumberFrames,
                             AudioBufferList				* ioData);

#ifdef __cplusplus
}
#endif


// redeclare readwrite for class continuation
#if defined ( USING_OSX )
AudioUnit outputUnit;
UInt32 numInputChannels;
float *inData;
AudioBufferList *inputBuffer;
#endif
AudioUnit inputUnit;
BOOL inputAvailable;
UInt32 numOutputChannels;
Float64 samplingRate;
NSTimeInterval bufferDuration;
BOOL isInterleaved;
UInt32 numBytesPerSample;

AudioStreamBasicDescription outputFormat;
BOOL playing;
float *outData;

#if defined (USING_OSX)
AudioDeviceID *deviceIDs;
NSMutableArray *deviceNames;
AudioDeviceID defaultInputDeviceID;
NSString *defaultInputDeviceName;
AudioDeviceID defaultOutputDeviceID;
NSString *defaultOutputDeviceName;
AudioFileWriter* fileWriter;
void enumerateAudioDevices();
#endif

NovocaineOutputBlock outputBlock;


void novocaine_init()
{
	
	// Initialize a float buffer to hold audio
	outData = (float *)calloc(8192, sizeof(float)); // probably more than we'll need
	outputBlock = nil;
	
#if defined ( USING_OSX )
	deviceNames = [[NSMutableArray alloc] initWithCapacity:100]; // more than we'll need
#endif
	
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
	
#elif defined ( USING_OSX )
	enumerateAudioDevices();
    inputAvailable = YES;
#endif
	
    // --- Audio Session Setup ---
    // ---------------------------
    
#if defined ( USING_IOS )
	
    // Add a property listener, to listen to changes to the session
    //CheckError( AudioSessionAddPropertyListener(kAudioSessionProperty_AudioRouteChange, sessionPropertyListener, (__bridge void*)self), "Couldn't add audio session property listener");
	
	//todo: doineedthis part#1
	/*
    [[NSNotificationCenter defaultCenter] addObserver: self
					     selector: @selector(handleRouteChange:)
						 name: AVAudioSessionRouteChangeNotification
					       object: session];
	*/
#if !TARGET_IPHONE_SIMULATOR
    //prevent runtime switching between 44.1 (headphones) and 48kHz (built-in speakers) on latest iPhone models with iOS9
    [session setPreferredSampleRate:48000. error:nil];
#endif
	
    novocaine_checkSessionProperties();
    
#endif
    
    // ----- Audio Unit Setup -----
    // ----------------------------
    
    
    // Describe the output unit.
    
#if defined ( USING_OSX )
    AudioComponentDescription inputDescription = {0};	
    inputDescription.componentType = kAudioUnitType_Output;
    inputDescription.componentSubType = kAudioUnitSubType_HALOutput;
    inputDescription.componentManufacturer = kAudioUnitManufacturer_Apple;
    
    AudioComponentDescription outputDescription = {0};	
    outputDescription.componentType = kAudioUnitType_Output;
    outputDescription.componentSubType = kAudioUnitSubType_HALOutput;
    outputDescription.componentManufacturer = kAudioUnitManufacturer_Apple;    
    
#elif defined (USING_IOS)
    AudioComponentDescription inputDescription = {0};	
    inputDescription.componentType = kAudioUnitType_Output;
    inputDescription.componentSubType = kAudioUnitSubType_RemoteIO;
    inputDescription.componentManufacturer = kAudioUnitManufacturer_Apple;
    
#endif
    
    
    
    // Get component
    AudioComponent inputComponent = AudioComponentFindNext(NULL, &inputDescription);
    CheckError( AudioComponentInstanceNew(inputComponent, &inputUnit), "Couldn't create the output audio unit");
    
#if defined ( USING_OSX )
    AudioComponent outputComponent = AudioComponentFindNext(NULL, &outputDescription);
    CheckError( AudioComponentInstanceNew(outputComponent, &outputUnit), "Couldn't create the output audio unit");
#endif

#if defined ( USING_OSX )    
    // Disable output on the input unit
    // (only on Mac, since on the iPhone, the input unit is also the output unit)
	UInt32 one = 1;
    UInt32 zero = 0;
    CheckError( AudioUnitSetProperty(inputUnit,
                                     kAudioOutputUnitProperty_EnableIO, 
                                     kAudioUnitScope_Output, 
                                     kOutputBus, 
                                     &zero, 
                                     sizeof(UInt32)), "Couldn't disable output on the audio unit");
    
    // Enable output
    CheckError( AudioUnitSetProperty(outputUnit,
                                     kAudioOutputUnitProperty_EnableIO, 
                                     kAudioUnitScope_Output, 
                                     kOutputBus, 
                                     &one, 
                                     sizeof(one)), "Couldn't enable IO on the input scope of output unit");
    
    // Disable input
    CheckError( AudioUnitSetProperty(outputUnit,
                                     kAudioOutputUnitProperty_EnableIO, 
                                     kAudioUnitScope_Input, 
                                     kInputBus, 
                                     &zero, 
                                     sizeof(UInt32)), "Couldn't disable output on the audio unit");
    
#endif
    
    // TODO: first query the hardware for desired stream descriptions
    // Check the input stream format
    
# if defined ( USING_IOS )
    UInt32 size;
	size = sizeof( AudioStreamBasicDescription );

	//TODO: is it necesary? Sampling rate already set!?
	// Check the output stream format
	size = sizeof( AudioStreamBasicDescription );
	CheckError( AudioUnitGetProperty(inputUnit,
                                     kAudioUnitProperty_StreamFormat, 
                                     kAudioUnitScope_Output, 
                                     1, 
                                     &outputFormat,
                                     &size ), 
               "Couldn't get the hardware output stream format");
	
	outputFormat.mSampleRate = samplingRate;
    numBytesPerSample = outputFormat.mBitsPerChannel / 8;
    
    size = sizeof(AudioStreamBasicDescription);
	CheckError(AudioUnitSetProperty(inputUnit,
									kAudioUnitProperty_StreamFormat,
									kAudioUnitScope_Output,
									kInputBus,
									&outputFormat,
									size),
			   "Couldn't set the ASBD on the audio unit (after setting its sampling rate)");
    
    
# elif defined ( USING_OSX )
    
    UInt32 size = sizeof(AudioDeviceID);
    if(defaultInputDeviceID == kAudioDeviceUnknown)
    {
        AudioObjectPropertyAddress propertyAddress;
        propertyAddress.mSelector = kAudioHardwarePropertyDefaultInputDevice;
        propertyAddress.mScope = kAudioObjectPropertyScopeGlobal;
        propertyAddress.mElement = kAudioObjectPropertyElementMaster;
        
        AudioDeviceID thisDeviceID;            
        UInt32 propsize = sizeof(AudioDeviceID);
        CheckError(AudioObjectGetPropertyData(kAudioObjectSystemObject, &propertyAddress, 0, NULL, &propsize, &thisDeviceID), "Could not get the default device");
        defaultInputDeviceID = thisDeviceID;
    }
    
    if (defaultOutputDeviceID == kAudioDeviceUnknown)
    {
        AudioObjectPropertyAddress propertyAddress;
        propertyAddress.mSelector = kAudioHardwarePropertyDefaultOutputDevice;
        propertyAddress.mScope = kAudioObjectPropertyScopeGlobal;
        propertyAddress.mElement = kAudioObjectPropertyElementMaster;
        
        AudioDeviceID thisDeviceID;            
        UInt32 propsize = sizeof(AudioDeviceID);
        CheckError(AudioObjectGetPropertyData(kAudioObjectSystemObject, &propertyAddress, 0, NULL, &propsize, &thisDeviceID), "Could not get the default device");
        defaultOutputDeviceID = thisDeviceID;
        
    }
    
    
    // Set the current device to the default input unit.
    CheckError( AudioUnitSetProperty( inputUnit, 
                                     kAudioOutputUnitProperty_CurrentDevice, 
                                     kAudioUnitScope_Global, 
                                     kOutputBus, 
                                     &defaultInputDeviceID,
                                     sizeof(AudioDeviceID) ), "Couldn't set the current input audio device");
    
    CheckError( AudioUnitSetProperty( outputUnit,
                                     kAudioOutputUnitProperty_CurrentDevice, 
                                     kAudioUnitScope_Global, 
                                     kOutputBus, 
                                     &defaultOutputDeviceID,
                                     sizeof(AudioDeviceID) ), "Couldn't set the current output audio device");
    
    
	UInt32 propertySize = sizeof(AudioStreamBasicDescription);
	CheckError(AudioUnitGetProperty(inputUnit,
									kAudioUnitProperty_StreamFormat,
									kAudioUnitScope_Output,
									kInputBus,
									&outputFormat,
									&propertySize),
			   "Couldn't get ASBD from input unit");
	
	numOutputChannels = outputFormat.mChannelsPerFrame;

	samplingRate = outputFormat.mSampleRate;
	numBytesPerSample = outputFormat.mBitsPerChannel / 8;
	
    propertySize = sizeof(AudioStreamBasicDescription);
	CheckError(AudioUnitSetProperty(inputUnit,
									kAudioUnitProperty_StreamFormat,
									kAudioUnitScope_Output,
									kInputBus,
									&outputFormat,
									propertySize),
			   "Couldn't set the ASBD on the audio unit (after setting its sampling rate)");
    
    
#endif
    
    
    
#if defined ( USING_IOS )
	//todo: ios: result not used!? test, remove if works without it
    UInt32 numFramesPerBuffer;
    size = sizeof(UInt32);
    CheckError(AudioUnitGetProperty(inputUnit,
                                    kAudioUnitProperty_MaximumFramesPerSlice,
                                    kAudioUnitScope_Global, 
                                    kOutputBus, 
                                    &numFramesPerBuffer, 
                                    &size), 
               "Couldn't get the number of frames per callback");
    
    UInt32 bufferSizeBytes = outputFormat.mBytesPerFrame * outputFormat.mFramesPerPacket * numFramesPerBuffer;
    
#elif defined ( USING_OSX )
	// Get the size of the IO buffer(s)
	UInt32 bufferSizeFrames = 0;
	size = sizeof(UInt32);
	CheckError (AudioUnitGetProperty(inputUnit,
									 kAudioDevicePropertyBufferFrameSize,
									 kAudioUnitScope_Global,
									 0,
									 &bufferSizeFrames,
									 &size),
				"Couldn't get buffer frame size from input unit");
	UInt32 bufferSizeBytes = bufferSizeFrames * sizeof(Float32);
	// Buffer duration on OSX
	bufferDuration = 1.0f/samplingRate*bufferSizeFrames;
	NSLog(@"IO buffer duration is %f", bufferDuration);

	if (outputFormat.mFormatFlags & kAudioFormatFlagIsNonInterleaved) {
        // The audio is non-interleaved
        NSLog(@"Not interleaved!\n");
        isInterleaved = NO;
        
        // allocate an AudioBufferList plus enough space for array of AudioBuffers
		UInt32 propsize = offsetof(AudioBufferList, mBuffers[0]) + (sizeof(AudioBuffer) * outputFormat.mChannelsPerFrame);
		
		//malloc buffer lists
		inputBuffer = (AudioBufferList *)malloc(propsize);
		inputBuffer->mNumberBuffers = outputFormat.mChannelsPerFrame;
		
		//pre-malloc buffers for AudioBufferLists
		for(UInt32 i =0; i< inputBuffer->mNumberBuffers ; i++) {
			inputBuffer->mBuffers[i].mNumberChannels = 1;
			inputBuffer->mBuffers[i].mDataByteSize = bufferSizeBytes;
			inputBuffer->mBuffers[i].mData = malloc(bufferSizeBytes);
            memset(inputBuffer->mBuffers[i].mData, 0, bufferSizeBytes);
		}
        
	} else {
		NSLog (@"Format is interleaved\n");
        isInterleaved = YES;
        
		// allocate an AudioBufferList plus enough space for array of AudioBuffers
		UInt32 propsize = offsetof(AudioBufferList, mBuffers[0]) + (sizeof(AudioBuffer) * 1);
		
		//malloc buffer lists
		inputBuffer = (AudioBufferList *)malloc(propsize);
		inputBuffer->mNumberBuffers = 1;
		
		//pre-malloc buffers for AudioBufferLists
		inputBuffer->mBuffers[0].mNumberChannels = outputFormat.mChannelsPerFrame;
		inputBuffer->mBuffers[0].mDataByteSize = bufferSizeBytes;
		inputBuffer->mBuffers[0].mData = malloc(bufferSizeBytes);
        memset(inputBuffer->mBuffers[0].mData, 0, bufferSizeBytes);
        
	}
#endif
    
    // Slap a render callback on the unit
    AURenderCallbackStruct callbackStruct;
    
    callbackStruct.inputProc = renderCallback;
# if defined ( USING_OSX )
    CheckError( AudioUnitSetProperty(outputUnit,
                                     kAudioUnitProperty_SetRenderCallback, 
                                     kAudioUnitScope_Input,
                                     0,
                                     &callbackStruct, 
                                     sizeof(callbackStruct)), 
               "Couldn't set the render callback on the input unit");
    
#elif defined ( USING_IOS )
    CheckError( AudioUnitSetProperty(inputUnit,
                                     kAudioUnitProperty_SetRenderCallback, 
                                     kAudioUnitScope_Input,
                                     0,
                                     &callbackStruct, 
                                     sizeof(callbackStruct)), 
               "Couldn't set the render callback on the input unit");    
#endif

	CheckError(AudioUnitInitialize(inputUnit), "Couldn't initialize the output unit");
#if defined ( USING_OSX )
    CheckError(AudioUnitInitialize(outputUnit), "Couldn't initialize the output unit");
#endif
    
}

#if defined (USING_OSX)
void enumerateAudioDevices()
{
    AudioObjectPropertyAddress propertyAddress;
    propertyAddress.mSelector = kAudioHardwarePropertyDefaultInputDevice;
    propertyAddress.mScope = kAudioObjectPropertyScopeGlobal;
    propertyAddress.mElement = kAudioObjectPropertyElementMaster;
    
    UInt32 propSize = sizeof(AudioDeviceID);
    CheckError(AudioObjectGetPropertyData(kAudioObjectSystemObject, &propertyAddress, 0, NULL, &propSize, &defaultInputDeviceID), "Could not get the default device");
    
    propertyAddress.mSelector = kAudioHardwarePropertyDevices;
    propertyAddress.mScope = kAudioObjectPropertyScopeGlobal;
    propertyAddress.mElement = kAudioObjectPropertyElementMaster;
    
    AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &propertyAddress, 0, NULL, &propSize);
    uint32_t deviceCount = ( propSize / sizeof(AudioDeviceID) );
    
    // Allocate the device IDs
    deviceIDs = (AudioDeviceID *)calloc(deviceCount, sizeof(AudioDeviceID));
    [deviceNames removeAllObjects];
    
    // Get all the device IDs
    CheckError( AudioObjectGetPropertyData(kAudioObjectSystemObject, &propertyAddress, 0, NULL, &propSize, deviceIDs ), "Could not get device IDs");
    
    
    // Get the names of all the device IDs
    // 256 chars should be big enough for pretty much any name
    char deviceNameBuffer[256];
    char mfrNameBuffer[256];
    UInt32 nameBufSize;
    AudioObjectPropertyAddress deviceAddress;
    
    for( int i = 0; i < deviceCount; i++ )
    {
        deviceAddress.mSelector = kAudioDevicePropertyDeviceName;
        deviceAddress.mScope = kAudioObjectPropertyScopeGlobal;
        deviceAddress.mElement = kAudioObjectPropertyElementMaster;
        
        nameBufSize = sizeof(deviceNameBuffer);
        
        CheckError( AudioObjectGetPropertyData(deviceIDs[i], &deviceAddress, 0, NULL, &nameBufSize, deviceNameBuffer), "Could not get device name");

        deviceAddress.mSelector = kAudioDevicePropertyDeviceManufacturer;
        deviceAddress.mScope = kAudioObjectPropertyScopeGlobal;
        deviceAddress.mElement = kAudioObjectPropertyElementMaster;
        
        nameBufSize = sizeof(mfrNameBuffer);
        
        CheckError( AudioObjectGetPropertyData(deviceIDs[i], &deviceAddress, 0, NULL, &nameBufSize, mfrNameBuffer), "Could not get device manufacturer");
        
        NSString *thisDeviceName = [NSString stringWithFormat:@"%@: %@", [NSString stringWithUTF8String:mfrNameBuffer], [NSString stringWithUTF8String:deviceNameBuffer]];
        NSLog(@"Device: %@, ID: %d", thisDeviceName, deviceIDs[i]);
        [deviceNames addObject:thisDeviceName];
    }
}

#endif



void novocaine_pause() {
	
	if (playing) {
        CheckError( AudioOutputUnitStop(inputUnit), "Couldn't stop the output unit");
#if defined ( USING_OSX )
		CheckError( AudioOutputUnitStop(outputUnit), "Couldn't stop the output unit");
#endif
		playing = NO;
	}
    
}

void novocaine_play() {
	
	UInt32 isInputAvailable=0;
    
#if defined ( USING_IOS )

    isInputAvailable = 1;
#elif defined ( USING_OSX )
    isInputAvailable = 1;
    
#endif
    
    inputAvailable = isInputAvailable;
    
	if ( inputAvailable ) {
		// Set the audio session category for simultaneous play and record
		if (!playing) {
			CheckError( AudioOutputUnitStart(inputUnit), "Couldn't start the output unit");
#if defined ( USING_OSX )
            CheckError( AudioOutputUnitStart(outputUnit), "Couldn't start the output unit");
#endif
            
            playing = YES;
            
		}
	}
    
}

OSStatus renderCallback (void						*inRefCon,
                         AudioUnitRenderActionFlags	* ioActionFlags,
                         const AudioTimeStamp 		* inTimeStamp,
                         UInt32						inOutputBusNumber,
                         UInt32						inNumberFrames,
                         AudioBufferList				* ioData)
{
    // autorelease pool for much faster ARC performance on repeated calls from separate thread
   // @autoreleasepool {
        
        //Novocaine *sm = (Novocaine *)inRefCon;
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
        if ( numBytesPerSample == 4 ) // then we've already got floats
        {
            
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
        }
/*        else if ( sm.numBytesPerSample == 2 ) // then we need to convert SInt16 -> Float (and also scale)
        {
            float scale = (float)INT16_MAX;
            vDSP_vsmul(sm.outData, 1, &scale, sm.outData, 1, inNumberFrames*sm.numOutputChannels);
            
            for (int iBuffer=0; iBuffer < ioData->mNumberBuffers; ++iBuffer) {
                
                int thisNumChannels = ioData->mBuffers[iBuffer].mNumberChannels;
                
                for (int iChannel = 0; iChannel < thisNumChannels; ++iChannel) {
                    
                    int interleaveOffset = iChannel;
                    if (iBuffer < sm.numOutputChannels){
                        interleaveOffset += iBuffer;
                    }
                    
                    vDSP_vfix16(sm.outData+interleaveOffset, sm.numOutputChannels, (SInt16 *)ioData->mBuffers[iBuffer].mData+iChannel, thisNumChannels, inNumberFrames);
                }
            }
            
        }
*/	    
    //}

    return noErr;
    
}	

//TODO: do i need this?
//#pragma mark - Audio Session Listeners
#if defined (USING_IOS)
/*
-(void)handleRouteChange:(NSNotification*)notification{
    // Determines the reason for the route change, to ensure that it is not
    //      because of a category change.
    NSInteger  reason = [[[notification userInfo] objectForKey:AVAudioSessionRouteChangeReasonKey] integerValue];

    if (reason != AVAudioSessionRouteChangeReasonCategoryChange)
    {
        novocaine_checkSessionProperties();
    }
}
*/

// To be run ONCE per session property change and once on initialization.
void novocaine_checkSessionProperties()
{
    AVAudioSession *session = [AVAudioSession sharedInstance];

    // Check the number of input channels.
    numOutputChannels = [session outputNumberOfChannels];
    NSLog(@"We've got %u output channels", (unsigned int)numOutputChannels);
    
    // Get the hardware sampling rate. This is settable, but here we're only reading.
    samplingRate = [session sampleRate];
    NSLog(@"Current sampling rate: %f", samplingRate);
	
    bufferDuration = [session IOBufferDuration];
    NSLog(@"IO buffer duration is %f", bufferDuration);
}

#endif

void novocaine_setOutputBlock(NovocaineOutputBlock block)
{
	outputBlock=block;
}

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
    fileWriter = nil;
    }
}
#endif








