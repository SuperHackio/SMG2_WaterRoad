# SMG2 WaterRoad

## WaterRoad in SMG2

This is the long awaited port of Sea Slide's WaterRoad to SMG2!

Below is a list of everything that's included:
- The WaterRoad object for placing down in a level editor.
- JSON files for particle effects.

Below is a list of other things that you will need:
- A level editor.
- A particle editor with support for JSON files.
- A UseResource with all of WaterRoad's sound labels.
> *Note: Vanilla SMG2 already has ActionSound entries for WaterRoad, but this port isn't yet setup to make use of these. For your UseResource setup, it is adviced to simply copy over all 11 labels from ActionSound.*

## Credits
- @mrl - Decompiled a majority of WaterRoad.
- @superhackio - Decompiled parts of WaterRoad. Ported and enhanced SMG1 code to SMG2.

## Setup
It is adviced to reference the [template mod](https://youtu.be/GSz7fjOaZf0) for this port. Regardless here are some parameters that apply to WaterRoad.

### WaterRoad
| Parameter | Description |
| --- | --- |
| CommonPath_ID | Path ID. The path that WaterRoad is supposed to be created on. |
| CameraSetId | Camera Set ID. The camera that's active while Mario is bound to WaterRoad. For example: if this is set to 5, the camera label would be `e:ウォーターロード固有バインド中005`. |
| Obj_arg0 | Allow reuse? If set, WaterRoad will not disappear after use. Off by default. |
| Obj_arg1 | Radius. The radius of the model. Default 500. |
| Obj_arg2 | Speed. The speed at which Mario travels through WaterRoad. Default 100. |
| Obj_arg3 | Speed up. The speed Mario briefly accelerates to when spinning inside WaterRoad. Default 150. |
| SW_A | Can be used in a cutscene to kill the object. Untested. |
| SW_AWAKE | Enables updates and rendering when activated. Can be used to show/hide WaterRoad. |

The appearance animation is controlled via a cutscene. Please reference the [template mod](https://youtu.be/GSz7fjOaZf0) for the setup.
