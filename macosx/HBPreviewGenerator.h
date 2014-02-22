/*  HBPreviewGenerator.h $

 This file is part of the HandBrake source code.
 Homepage: <http://handbrake.fr/>.
 It may be used under the terms of the GNU General Public License. */

#import <Cocoa/Cocoa.h>
#include "hb.h"

@protocol HBPreviewGeneratorDelegate <NSObject>

- (void) didCreateMovieAtURL: (NSURL *) fileURL;
- (void) updateProgress: (double) progress info: (NSString *) progressInfo;

@end

@interface HBPreviewGenerator : NSObject

@property (nonatomic, assign) id <HBPreviewGeneratorDelegate> delegate;
@property (nonatomic) BOOL deinterlace;

- (id) initWithHandle: (hb_handle_t *) handle andTitle: (hb_title_t *) title;

/* Still image generator */
- (NSImage *) imageAtIndex: (NSUInteger) index shouldCache: (BOOL) cache;
- (NSUInteger) imagesCount;
- (void) purgeImageCache;

/* Video generator */
- (BOOL) createMovieAsyncWithImageIndex: (NSUInteger) index andDuration: (NSUInteger) duration;
- (void) cancel;

@end
