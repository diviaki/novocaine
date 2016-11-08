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

@interface Novocaine : NSObject

// ------ These properties/methods are used for configuration -------

@property (nonatomic, copy)     NSString *inputRoute;

// Explicitly declaring the block setters will create the correct block signature for auto-complete.
// These will map to the setters for the block properties below.
#if defined ( USING_OSX )
- (void)setInputBlock:(NovocaineInputBlock)block;
#endif
- (void)setOutputBlock:(NovocaineOutputBlock)block;

#if defined ( USING_OSX )
@property (nonatomic, copy) NovocaineInputBlock inputBlock;
#endif
@property (nonatomic, copy) NovocaineOutputBlock outputBlock;

// ------------------------------------------------------------------

// these should be readonly in public interface - no need for public write access
#if defined ( USING_OSX )
@property (nonatomic, assign, readonly) AudioUnit outputUnit;
@property (nonatomic, assign, readonly) UInt32 numInputChannels;
@property (nonatomic, assign, readonly) AudioBufferList *inputBuffer;
#endif
@property (nonatomic, assign, readonly) AudioUnit inputUnit;
@property (nonatomic, assign, readonly) BOOL inputAvailable;
@property (nonatomic, assign, readonly) UInt32 numOutputChannels;
@property (nonatomic, assign, readonly) Float64 samplingRate;
@property (nonatomic, assign, readonly) NSTimeInterval bufferDuration;
@property (nonatomic, assign, readonly) BOOL isInterleaved;
@property (nonatomic, assign, readonly) UInt32 numBytesPerSample;
#ifdef USE_MICROPHONE
@property (nonatomic, assign, readonly) AudioStreamBasicDescription inputFormat;
#endif
@property (nonatomic, assign, readonly) AudioStreamBasicDescription outputFormat;
@property (nonatomic, assign, readonly) BOOL playing;


// Singleton methods
+ (Novocaine *) audioManager;

// Audio Unit methods
- (void)play;
- (void)pause;

#if defined ( USING_OSX )
- (void)recordOutput:(NSString*)filename;
#endif

#if defined ( USING_IOS )
- (void)checkSessionProperties;
#endif


@end
