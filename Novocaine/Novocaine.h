#import <CoreFoundation/CoreFoundation.h>
#import <AudioToolbox/AudioToolbox.h>
#import <Accelerate/Accelerate.h>

#import "TargetConditionals.h"

#if TARGET_OS_IPHONE
    #define USING_IOS
    #include <AVFoundation/AVFoundation.h>
#else
    #define USING_OSX 
    #include <CoreAudio/CoreAudio.h>
#endif

#if defined USING_OSX
#include "AudioFileWriter.h"
#endif

typedef void (^NovocaineOutputBlock)(float *data, UInt32 numFrames, UInt32 numChannels);

// ------ These properties/methods are used for configuration -------

extern Float64 samplingRate;

#ifdef __cplusplus
extern "C" {
#endif

void novocaine_init();
void novocaine_setOutputBlock(NovocaineOutputBlock block);
void novocaine_play();
void novocaine_pause();

#if defined ( USING_OSX )
void novocaine_recordOutput(NSString* filename);
#endif

#if defined ( USING_IOS )
void novocaine_checkSessionProperties();
#endif
	
#ifdef __cplusplus
}
#endif

