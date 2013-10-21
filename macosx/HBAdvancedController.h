/* HBAdvancedController

    This file is part of the HandBrake source code.
    Homepage: <http://handbrake.fr/>.
    It may be used under the terms of the GNU General Public License. */

#import <Cocoa/Cocoa.h>

@interface HBAdvancedController : NSViewController
{
    /* Advanced Tab for opts fX264optView*/
    IBOutlet NSView             * fX264optView;
    IBOutlet NSView             * fEmptyView;
    IBOutlet NSTextField        * fX264optViewTitleLabel;
    IBOutlet NSTextField        * fDisplayX264OptionsLabel;
    IBOutlet NSTextField        * fDisplayX264Options;
    IBOutlet NSTextField        * fDisplayLavcOptionsLabel;
    IBOutlet NSTextField        * fDisplayLavcOptions;
    
    IBOutlet NSTextField        * fDisplayTheoraOptionsLabel;
    
    IBOutlet NSTextField        * fX264optBframesLabel;
    IBOutlet NSPopUpButton      * fX264optBframesPopUp;
    IBOutlet NSTextField        * fX264optRefLabel;
    IBOutlet NSPopUpButton      * fX264optRefPopUp;
    IBOutlet NSButton           * fX264optWeightPSwitch;
    IBOutlet NSTextField        * fX264optWeightPLabel;
    IBOutlet NSTextField        * fX264optNodctdcmtLabel;
    IBOutlet NSButton           * fX264optNodctdcmtSwitch;
    IBOutlet NSTextField        * fX264optSubmeLabel;
    IBOutlet NSPopUpButton      * fX264optSubmePopUp;
    IBOutlet NSTextField        * fX264optTrellisLabel;
    IBOutlet NSPopUpButton      * fX264optTrellisPopUp;
    IBOutlet NSTextField        * fX264optMotionEstLabel;
    IBOutlet NSPopUpButton      * fX264optMotionEstPopUp;
    IBOutlet NSTextField        * fX264optMERangeLabel;
    IBOutlet NSPopUpButton      * fX264optMERangePopUp;
    IBOutlet NSTextField        * fX264optBPyramidLabel;
    IBOutlet NSPopUpButton      * fX264optBPyramidPopUp;
    IBOutlet NSTextField        * fX264optDirectPredLabel;
    IBOutlet NSPopUpButton      * fX264optDirectPredPopUp;
    IBOutlet NSTextField        * fX264optDeblockLabel;
    IBOutlet NSPopUpButton      * fX264optAlphaDeblockPopUp;
    IBOutlet NSPopUpButton      * fX264optBetaDeblockPopUp;
    IBOutlet NSTextField        * fX264optAnalyseLabel;
    IBOutlet NSPopUpButton      * fX264optAnalysePopUp;
    IBOutlet NSTextField        * fX264opt8x8dctLabel;
    IBOutlet NSButton           * fX264opt8x8dctSwitch;
    IBOutlet NSTextField        * fX264optCabacLabel;
    IBOutlet NSButton           * fX264optCabacSwitch;
    IBOutlet NSSlider           * fX264optAqSlider;
    IBOutlet NSTextField        * fX264optAqLabel;
    IBOutlet NSSlider           * fX264optPsyRDSlider;
    IBOutlet NSTextField        * fX264optPsyRDLabel;
    IBOutlet NSSlider           * fX264optPsyTrellisSlider;
    IBOutlet NSTextField        * fX264optPsyTrellisLabel;
    IBOutlet NSPopUpButton      * fX264optBAdaptPopUp;
    IBOutlet NSTextField        * fX264optBAdaptLabel;
}

// x264 Advanced Panel Methods
- (NSString *) optionsString;
- (NSString *) optionsStringLavc;
- (void) setOptions: (NSString *)string;
- (void) setLavcOptions: (NSString *)string;
- (void) enableUI: (bool) b;
- (void) showAdvancedX264Options: (BOOL) show;
- (void) setLavcOptsEnabled: (BOOL) lavc;
- (void) setVideoEncoderName: (NSString *)encoderName;
- (IBAction) X264AdvancedOptionsAnimate: (id) sender;
- (IBAction) X264AdvancedOptionsSet: (id) sender;
- (IBAction) X264AdvancedOptionsStandardizeOptString: (id) sender;
- (IBAction) X264AdvancedOptionsSetCurrentSettings: (id) sender;
- (NSString *)  X264AdvancedOptionsStandardizeOptNames:(NSString *) cleanOptNameString;
- (NSString *)  X264AdvancedOptionsOptIDToString: (id) sender;
- (NSString *)  X264AdvancedOptionsWidgetToString: (NSString *) optName withID: (id) sender;
- (BOOL) X264AdvancedOptionsIsOpt: (NSString *) optNameToChange inString: (NSString *) currentOptString;
- (IBAction) X264AdvancedOptionsChanged: (id) sender;

@end
