## IMPORTS
import json

## CONSTRANTS
HALF_FIELD_WIDTH = 8.0 / 2.0

## INVERT POINT
def invertPt(pt):
        return {
            "x": pt["x"] + (2.0 * (HALF_FIELD_WIDTH - pt["x"])),
            "y": pt["y"]
        }
        #return {
        #    "x": pt["x"],
        #    "y": pt["y"] + (2.0 * (HALF_FIELD_WIDTH - pt["y"]))
        #}

## INVERT ANGLE
def invertAngle(angle):
        return angle + (2 * (90.0 - angle))

## RUN
def invertPathPlannerFile(path, outputPath):
        # open file
        with open(path, "r") as file:
                traj = json.load(file)

                # invert waypoints
                for wp in traj["waypoints"]:
                    wp["anchorPoint"] = invertPt(wp["anchorPoint"])

                    wp["prevControl"] = invertPt(wp["prevControl"]) if (wp["prevControl"] != None) else None
                    wp["nextControl"] = invertPt(wp["nextControl"]) if (wp["nextControl"] != None) else None

                    wp["holonomicAngle"] = invertAngle(wp["holonomicAngle"])

                # save new trajectory
                with open(outputPath, "w+") as outputFile:
                    json.dump(traj, outputFile, indent=4)
                    
        print("Done with " + path)

## RUN
while True:
    invertPathPlannerFile(
        input("Input file name -> "),
        input("Output file name -> ")
    )
