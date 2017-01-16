// Copyright (c) 2012 Alex Wiltschko
// 
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following
// conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// TODO:
// Switching mic and speaker on/off
//
// HOUSEKEEPING AND NICE FEATURES:
// Disambiguate outputFormat (the AUHAL's stream format)
// More nuanced input detection on the Mac
// Route switching should work, check with iPhone
// Device switching should work, check with laptop. Read that damn book.
// Wrap logging with debug macros.
// Think about what should be public, what private.
// Ability to select non-default devices.


#include "Novocaine.h"
#define kInputBus 1
#define kOutputBus 0
#define kDefaultDevice 999999

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
    
#if OPT_USE_MICROPHONE
    OSStatus inputCallback (void						*inRefCon,
                            AudioUnitRenderActionFlags	* ioActionFlags,
                            const AudioTimeStamp 		* inTimeStamp,
                            UInt32						inOutputBusNumber,
                            UInt32						inNumberFrames,
                            AudioBufferList				* ioData);
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
#ifdef USE_MICROPHONE
AudioStreamBasicDescription inputFormat;
#endif
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

// must be called prior to playing audio
void setupAudioSession();
void setupAudioUnits();

#if defined ( USING_OSX )
NovocaineInputBlock inputBlock;
#endif
NovocaineOutputBlock outputBlock;


#pragma mark - Singleton Methods

void novocaine_init()
{
	
	// Initialize a float buffer to hold audio
#if defined (USING_OSX)
	inData  = (float *)calloc(8192, sizeof(float)); // probably more than we'll need
	inputBlock = nil;
#endif
	outData = (float *)calloc(8192, sizeof(float));
	outputBlock = nil;
	
#if defined ( USING_OSX )
	deviceNames = [[NSMutableArray alloc] initWithCapacity:100]; // more than we'll need
#endif
	
	playing = NO;
	
	// Fire up the audio session ( with steady error checking ... )
	setupAudioSession();
	
	// start audio units
	setupAudioUnits();
}

void dealloc() //todo call from app.dealloc (or skip altogether)
{
	free(outData);

#if defined (USING_OSX)
    free(inData);

	if (deviceIDs){
        free(deviceIDs);
    }
	
	//freebuffers
	if (inputBuffer){
		
		for(UInt32 i =0; i< inputBuffer->mNumberBuffers ; i++) {
			
			if(inputBuffer->mBuffers[i].mData){
				free(inputBuffer->mBuffers[i].mData);
			}
		}
		
		free(inputBuffer);
		inputBuffer = NULL;
	}
#endif
}

#pragma mark - Audio Methods


void setupAudioSession()
{
    // Initialize and configure the audio session, and add an interuption listener
    
#if defined ( USING_IOS )

    AVAudioSession *session = [AVAudioSession sharedInstance];

    NSError *setCategoryError = nil;
    if (![session setCategory:
#if OPT_USE_MICROPHONE
          AVAudioSessionCategoryPlayAndRecord
#else
          AVAudioSessionCategoryPlayback
#endif
#if DEBUG   //help developers listen to music during development :)
                withOptions:AVAudioSessionCategoryOptionMixWithOthers
#endif
                error:&setCategoryError]) {
        // handle error
    }

    // Set the audio session active
    NSError *err = nil;
    if (![[AVAudioSession sharedInstance] setActive:YES error:&err]){
        NSLog(@"Couldn't activate audio session: %@", err);
    }
#if OPT_USE_MICROPHONE
    checkAudioSource();
#endif
#elif defined ( USING_OSX )
    // TODO: grab the audio
	enumerateAudioDevices();
    inputAvailable = YES;
#endif
}


void setupAudioUnits()
{
    
    // --- Audio Session Setup ---
    // ---------------------------
    
#if defined ( USING_IOS )
	
    AVAudioSession *session = [AVAudioSession sharedInstance];
	
    NSError *error = nil;
		
    // Add a property listener, to listen to changes to the session
    //CheckError( AudioSessionAddPropertyListener(kAudioSessionProperty_AudioRouteChange, sessionPropertyListener, (__bridge void*)self), "Couldn't add audio session property listener");
	
	//todo: doineedthis part#1
	/*
    [[NSNotificationCenter defaultCenter] addObserver: self
					     selector: @selector(handleRouteChange:)
						 name: AVAudioSessionRouteChangeNotification
					       object: session];
	*/
    // Set the buffer size, this will affect the number of samples that get rendered every time the audio callback is fired
    // A small number will get you lower latency audio, but will make your processor work harder
#if !TARGET_IPHONE_SIMULATOR
    Float32 preferredBufferSize = 0.02322; //default on iOS9@iPhone5
	
    [session setPreferredIOBufferDuration:preferredBufferSize error:&error];
	
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
    
#if defined ( USING_OSX ) || defined ( OPT_USE_MICROPHONE )
    UInt32 one = 1;
#endif

    // Enable input
    // TODO: Conditionally disable input if option has not been specified
#if OPT_USE_MICROPHONE
    //calling this pops up the "this app would like to acces your microphone, allow/deny" dialog on ios 7+
    //only happens the very first time the app launched on a device - note that the setting survives app uninstall!

    //on iOS emitting sounds work without enabling kAudioUnitScope_Output
    CheckError( AudioUnitSetProperty(inputUnit,
                                     kAudioOutputUnitProperty_EnableIO,
                                     kAudioUnitScope_Input,
                                     kInputBus,
                                     &one,
                                     sizeof(one)), "Couldn't enable IO on the input scope of output unit");
#endif
    
#if defined ( USING_OSX )    
    // Disable output on the input unit
    // (only on Mac, since on the iPhone, the input unit is also the output unit)
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
#ifdef USE_MICROPHONE
	CheckError( AudioUnitGetProperty(inputUnit,
                                     kAudioUnitProperty_StreamFormat, 
                                     kAudioUnitScope_Input, 
                                     1, 
                                     &inputFormat,
                                     &size ),
               "Couldn't get the hardware input stream format");
#endif
	// Check the output stream format
	size = sizeof( AudioStreamBasicDescription );
	CheckError( AudioUnitGetProperty(inputUnit,
                                     kAudioUnitProperty_StreamFormat, 
                                     kAudioUnitScope_Output, 
                                     1, 
                                     &outputFormat,
                                     &size ), 
               "Couldn't get the hardware output stream format");
	
#ifdef USE_MICROPHONE
    inputFormat.mSampleRate = samplingRate;
#endif
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
    
    
#ifdef USE_MICROPHONE
	CheckError(AudioUnitGetProperty(inputUnit,
									kAudioUnitProperty_StreamFormat,
									kAudioUnitScope_Input,
									kInputBus,
									&inputFormat,
									&propertySize),
			   "Couldn't get ASBD from input unit");
    
    
    outputFormat.mSampleRate = inputFormat.mSampleRate;
	
	numInputChannels = inputFormat.mChannelsPerFrame;
#endif
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
#if OPT_USE_MICROPHONE
    callbackStruct.inputProc = inputCallback;
    callbackStruct.inputProcRefCon = (__bridge void *)(self);
    
    CheckError( AudioUnitSetProperty(inputUnit,
                                     kAudioOutputUnitProperty_SetInputCallback, 
                                     kAudioUnitScope_Global,
                                     0, 
                                     &callbackStruct, 
                                     sizeof(callbackStruct)), "Couldn't set the callback on the input unit");
#endif
    
    callbackStruct.inputProc = renderCallback;
    //callbackStruct.inputProcRefCon = (__bridge void *)(self);
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
#if OPT_USE_MICROPHONE
    UInt32 size = sizeof(isInputAvailable);
	CheckError( AudioSessionGetProperty(kAudioSessionProperty_AudioInputAvailable, 
                                        &size, 
                                        &isInputAvailable), "Couldn't check if input was available");
#else
    isInputAvailable = 1;
#endif
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


//#pragma mark - Render Methods
#if OPT_USE_MICROPHONE
OSStatus inputCallback   (void						*inRefCon,
                          AudioUnitRenderActionFlags	* ioActionFlags,
                          const AudioTimeStamp 		* inTimeStamp,
                          UInt32						inOutputBusNumber,
                          UInt32						inNumberFrames,
                          AudioBufferList			* ioData)
{
//    @autoreleasepool {
    
        Novocaine *sm = (Novocaine *)inRefCon;
        
        if (!sm.playing)
            return noErr;
        if (sm.inputBlock == nil)
            return noErr;    
        
        
        // Check the current number of channels		
        // Let's actually grab the audio
#if TARGET_IPHONE_SIMULATOR
        // this is a workaround for an issue with core audio on the simulator, //
        //  likely due to 44100 vs 48000 difference in OSX //
        if( inNumberFrames == 471 )
            inNumberFrames = 470;
#endif
        CheckError( AudioUnitRender(sm.inputUnit, ioActionFlags, inTimeStamp, inOutputBusNumber, inNumberFrames, sm.inputBuffer), "Couldn't render the output unit");
        
        
        // Convert the audio in something manageable
        // For Float32s ... 
        if ( sm.numBytesPerSample == 4 ) // then we've already got flaots
        {
            
            float zero = 0.0f;
            if ( ! sm.isInterleaved ) { // if the data is in separate buffers, make it interleaved
                for (int i=0; i < sm.numInputChannels; ++i) {
                    vDSP_vsadd((float *)sm.inputBuffer->mBuffers[i].mData, 1, &zero, sm.inData+i, 
                               sm.numInputChannels, inNumberFrames);
                }
            } 
            else { // if the data is already interleaved, copy it all in one happy block.
                // TODO: check mDataByteSize is proper 
                memcpy(sm.inData, (float *)sm.inputBuffer->mBuffers[0].mData, sm.inputBuffer->mBuffers[0].mDataByteSize);
            }
        }
        
        // For SInt16s ...
        else if ( sm.numBytesPerSample == 2 ) // then we're dealing with SInt16's
        {
            if ( ! sm.isInterleaved ) {
                for (int i=0; i < sm.numInputChannels; ++i) {
                    vDSP_vflt16((SInt16 *)sm.inputBuffer->mBuffers[i].mData, 1, sm.inData+i, sm.numInputChannels, inNumberFrames);
                }            
            }
            else {
                vDSP_vflt16((SInt16 *)sm.inputBuffer->mBuffers[0].mData, 1, sm.inData, 1, inNumberFrames*sm.numInputChannels);
            }
            
            float scale = 1.0 / (float)INT16_MAX;
            vDSP_vsmul(sm.inData, 1, &scale, sm.inData, 1, inNumberFrames*sm.numInputChannels);
        }
        
        // Now do the processing! 
        sm.inputBlock(sm.inData, inNumberFrames, sm.numInputChannels);
        
   // }
    
    return noErr;
	
	
}
#endif

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

#if OPT_USE_MICROPHONE
void checkAudioSource() {
    // Check what the incoming audio route is.
    UInt32 propertySize = sizeof(CFStringRef);
    CFStringRef route;
    CheckError( AudioSessionGetProperty(kAudioSessionProperty_AudioRoute, &propertySize, &route), "Couldn't check the audio route");
	NSString *inputRoute = (__bridge NSString *)route;
    CFRelease(route);
    NSLog(@"AudioRoute: %@", inputRoute);
    
    
    // Check if there's input available.
    // TODO: check if checking for available input is redundant.
    //          Possibly there's a different property ID change?
    UInt32 isInputAvailable = 0;
    UInt32 size = sizeof(isInputAvailable);
    CheckError( AudioSessionGetProperty(kAudioSessionProperty_AudioInputAvailable, 
                                        &size, 
                                        &isInputAvailable), "Couldn't check if input is available");
    inputAvailable = (BOOL)isInputAvailable;
    NSLog(@"Input available? %d", inputAvailable);
    
}
#endif

// To be run ONCE per session property change and once on initialization.
void novocaine_checkSessionProperties()
{
    AVAudioSession *session = [AVAudioSession sharedInstance];
#if OPT_USE_MICROPHONE
    // Check if there is input, and from where
    checkAudioSource();
    // Check the number of input channels.
    numInputChannels = [session inputNumberOfChannels];
    NSLog(@"We've got %u input channels", (unsigned int)numInputChannels);
#endif
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








