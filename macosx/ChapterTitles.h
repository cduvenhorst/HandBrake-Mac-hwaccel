/*  ChapterTitles.h $

   This file is part of the HandBrake source code.
   Homepage: <http://handbrake.fr/>.
   It may be used under the terms of the GNU General Public License. */

#include <Cocoa/Cocoa.h>
#include "hb.h"

@interface ChapterTitles : NSObject <NSTableViewDelegate,NSTableViewDataSource> {
    hb_title_t     *fTitle;
    NSMutableArray *fChapterTitlesArray;
}

// Trigger a refresh of data
- (void)resetWithTitle:(hb_title_t *)title;

// Get the list of chapter titles
- (NSArray*)chapterTitlesArray;

// Table View Delegates
- (NSUInteger)numberOfRowsInTableView:(NSTableView *)aTableView;

- (id)tableView:(NSTableView *)aTableView
      objectValueForTableColumn:(NSTableColumn *)aTableColumn
      row:(NSInteger)rowIndex;
      
- (void)tableView:(NSTableView *)aTableView
        setObjectValue:(id)anObject
        forTableColumn:(NSTableColumn *)aTableColumn
        row:(NSInteger)rowIndex;
@end
