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
extern NovocaineOutputBlock outputBlock;

#ifdef __cplusplus
extern "C" {
#endif

	void novocaine_init(void);
static inline void novocaine_setOutputBlock(NovocaineOutputBlock block){outputBlock=block;}
void novocaine_play(void);
void novocaine_pause(void);
	
#if defined ( USING_OSX )
void novocaine_recordOutput(NSString* filename);
#endif

#if defined ( USING_IOS )
void novocaine_checkSessionProperties(void);
void novocaine_routechange(NSNotification* n);
#endif
	
#ifdef __cplusplus
}
#endif

