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

#import <CoreFoundation/CoreFoundation.h>
#import <AudioToolbox/AudioToolbox.h>
#import <Accelerate/Accelerate.h>

#import "TargetConditionals.h"

#if TARGET_OS_IPHONE
    #define USING_IOS
    #include <AVFoundation/AVFoundation.h>
    #import <UIKit/UIKit.h>
#else
    #define USING_OSX 
    #include <CoreAudio/CoreAudio.h>
#endif

#include "AudioFileWriter.h"

#include <Block.h>

FOUNDATION_EXTERN void CheckError(OSStatus error, const char *operation);

#if defined ( USING_OSX )
typedef void (^NovocaineInputBlock)(float *data, UInt32 numFrames, UInt32 numChannels);
#endif
typedef void (^NovocaineOutputBlock)(float *data, UInt32 numFrames, UInt32 numChannels);

// ------ These properties/methods are used for configuration -------

extern Float64 samplingRate;

#ifdef __cplusplus
extern "C" {
#endif

// Explicitly declaring the block setters will create the correct block signature for auto-complete.
// These will map to the setters for the block properties below.
#if defined ( USING_OSX )
void novocaine_setInputBlock(NovocaineInputBlock block);
#endif
void novocaine_setOutputBlock(NovocaineOutputBlock block);

// ------------------------------------------------------------------

// Audio Unit methods
void novocaine_init();
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

