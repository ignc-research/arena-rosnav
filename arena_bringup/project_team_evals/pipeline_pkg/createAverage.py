import os
from matplotlib import image
from numpy import empty
import pandas as pd
import glob
import warnings
import pathlib as pl
from argparse import ArgumentParser

warnings.simplefilter(action='ignore', category=FutureWarning)

class CSVFormat:
    
    robotList = ["burger", "jackal","ridgeback","agvota","rto","rto_real"]
    plannerList = ["teb","dwa","mpc","rlca","arena","rosnav"]
    basicColumnList = ["time","done_reason","collision", "robot_radius", "robot_max_speed","number_dynamic_obs","number_static_obs"]
    notUsedColumnList = ["laser_scan", "robot_lin_vel_x", "robot_lin_vel_y","robot_ang_vel","robot_orientation","robot_pos_x","robot_pos_y","action",
                   "episode", "form_dynamic_obs", "form_static_obs", 
                   "local_planner","robot_model","map"]
    arrayColumns = ["size_dynamic_obs", "speed_dynamic_obs","size_static_obs"]
 
    def returnOriginalCSVColumnList():
        
        originalCSVColumnList =  CSVFormat.basicColumnList + CSVFormat.notUsedColumnList + CSVFormat.arrayColumns
        return originalCSVColumnList
    
    def returnNotUsedColumnList():
        return CSVFormat.notUsedColumnList
    
    def returnOutputColumnList():
        lst = list(range(0,15))
        append_str = "speed_dynamic_obs_"
        speed_dynamic_obs_column = [append_str + str(sub) for sub in lst]
        append_str = "size_dynamic_obs_"
        size_dynamic_obs_column = [append_str + str(sub) for sub in lst]
        append_str = "size_static_obs_"
        size_static_obs_column = [append_str + str(sub) for sub in lst]
        return CSVFormat.basicColumnList + CSVFormat.robotList + CSVFormat.plannerList + size_static_obs_column + size_dynamic_obs_column + speed_dynamic_obs_column
    
    def returnOutputDataFrame():
        columnList = CSVFormat.returnOutputColumnList()
        df = pd.DataFrame(columns=columnList)
        return df

class RecordedAverage:
    currentMapName = None
    currentRobot = None
    episodeAverages = pd.DataFrame()

    def createAverages(args):
        averagesCSVOutput = pd.DataFrame()

        pathToCSV = args.csv_path
        pathToImageFolder = args.image_path
        
        print("path to the directory containing the CSV files:", pathToCSV)  
        
        # stored the path where the output should be written to
        outputPath = "{}/CSVaverages/".format(pathToCSV)
        print("outputpath",outputPath)

        # creates the output path in the file system in case it does not exist yet
        path = pl.Path(outputPath)
        path.mkdir(parents=True, exist_ok=True)

        # iterates through all csv files in the specified path
        for fileName in glob.glob('{}/*.csv'.format(pathToCSV)):
            print("Currently working on:",fileName)
            RecordedAverage.episodeAverages = CSVFormat.returnOutputDataFrame()

            # reads the csv file and stores it as a data frame
            data = pd.read_csv(fileName)

            # calls a function to check the csv files. In case the csv file does not meet the required characteristics it will be ignored
            if(not(RecordedAverage.checkCSV(data))):
                continue

            RecordedAverage.createEpisodeAverage(data)
        
            csvAverage = RecordedAverage.calculateCSVAverage(RecordedAverage.episodeAverages)
            averagesCSVOutput = averagesCSVOutput.append(csvAverage)
  
        if (args.run_wcs == True):
            command = "python3 $(pwd)/world_complexity.py --folders_path {}".format(pathToImageFolder)
            print(command)
            os.system(command)


        worldComplexityData = pd.read_csv("{}/map_worldcomplexity_results.csv".format(pathToImageFolder))
        
        RecordedAverage.combineAveragesAndWorldComplexity(averagesCSVOutput,worldComplexityData, outputPath)

    def cleanWorldComplexityData(worldComplexityData):
        worldComplexityData[" distance_avg"] = worldComplexityData[" distance_avg"].replace("None","0")
        worldComplexityData[" distance_var"] = worldComplexityData[" distance_var"].replace("[]","0")
        worldComplexityData[" distance_norm"] = worldComplexityData[" distance_norm"].replace("[]","0")
        return worldComplexityData

    def combineAveragesAndWorldComplexity(averagesData, worldComplexityData, outputPath):
        
        worldComplexityData = RecordedAverage.cleanWorldComplexityData(worldComplexityData)

        averagesData=averagesData.reset_index()
        averagesData=averagesData.drop(columns="index")
        combinedDataFrame = pd.DataFrame()

        for index, row in averagesData.iterrows():
            print(row)
            averagesRow = row
            averagesIndex = index
            found = False
            for index, row in worldComplexityData.iterrows():
                worldComplexityRow = row
                worldComplexityIndex = index
                if(averagesRow["map"]==worldComplexityRow["World"]):
                    found = True
                    df1 = averagesData.iloc[[averagesIndex]]
                    df2 = worldComplexityData.iloc[[worldComplexityIndex]]
                    df1 = df1.reset_index(drop=True)
                    df2 = df2.reset_index(drop=True)
                    mergeddf = pd.concat([df1, df2], axis=1)
                    combinedDataFrame= combinedDataFrame.append(mergeddf)
            if(found == False):
                print(averagesRow["map"],"was not found")    
        
        # drop columns not necessary for the NN
        #combinedDataFrame = combinedDataFrame.drop(columns=["robot_model", "map", "number_dynamic_obs", "number_static_obs"])

        print("Output path: " + outputPath)

        combinedDataFrame=combinedDataFrame.rename(columns={
            "time": "episode_duration",
            "done_reason": "success_rate",
            "collision" :"collision_rate"
            })


        csvFilename = "{}/CombinedAverages.csv".format(outputPath)
        with open(csvFilename, 'w') as f:
            combinedDataFrame.to_csv(f, mode='w', header=f.tell()==0, index=False)

    def calculateCSVAverage(dataFrame):
        global currentMapName
        global currentRobot
        average = pd.DataFrame(dataFrame.mean().to_dict(),index=[dataFrame.index.values[-1]])
        average["map"] = currentMapName
        average["robot_model"] = currentRobot
        return average

    # creates a data frames which only contains the entries of one episode and calls the averageing function for that episode
    def createEpisodeAverage(data):
        fristEpisodeNr = data.loc[0][0]
        # create copy of data
        dataSubsequentEpisode = data.copy()
        # create dataframe with only the frist episode
        data.drop(data[data["episode"]!= fristEpisodeNr].index, inplace = True)
        # create dataframe with only the subsequent episodes
        dataSubsequentEpisode.drop(dataSubsequentEpisode[dataSubsequentEpisode["episode"] == fristEpisodeNr].index, inplace = True)
        newAverage = RecordedAverage.calculateEpisodeAverage(data)
        RecordedAverage.episodeAverages = RecordedAverage.episodeAverages.append(newAverage)

        # recursiv call for futher episodes
        if not(dataSubsequentEpisode.empty):
            dataSubsequentEpisode = dataSubsequentEpisode.reset_index()
            dataSubsequentEpisode = dataSubsequentEpisode.drop(columns = ["index"])
            RecordedAverage.createEpisodeAverage(dataSubsequentEpisode)


    # extracts an Array contained in a CSV column and return the array as a data frame
    def extractArray(dataFrame, columnName):

        pattern  ='|'.join(["\[\'None\'\]", "\[\'\'\]"])
        dataFrame[columnName] = dataFrame[columnName].str.replace(pattern,"0")
    
        # Creating own dataFrame for index sizedynamic_obs
    
        # Extracting size_dynamic_obs column out of Data Frame "data"
        extractedColumn = dataFrame[columnName].str.split(",",expand=True)
        # Creating Data Frame out of extracted data
        dataFrame_extractedColumn = pd.DataFrame(extractedColumn)

        # adds the column name as a prefix for the columns in the extracted data frame
        dataFrame_extractedColumn = dataFrame_extractedColumn.add_prefix(columnName+"_")

        
        patternNew  ='|'.join(["\[","\]","\'"])
        
        for col in dataFrame_extractedColumn:
            # deleting unwanted characters
            dataFrame_extractedColumn[col] = dataFrame_extractedColumn[col].str.replace(patternNew,"")
            # Changing type String to type float for every column
            dataFrame_extractedColumn[col] = dataFrame_extractedColumn[col].astype(float)
        return dataFrame_extractedColumn        
        

    def calculateEpisodeAverage(data):

        averageDataFrameExtended = CSVFormat.returnOutputDataFrame()
        global currentMapName
        global currentRobot

        #save important columns that will be removed
        currentRobot = data["robot_model"][0]
        currentMapName = data["map"][0]
        local_planner = data["local_planner"][0]

        # Drop unnecessary columns
        data = data.drop(columns=CSVFormat.returnNotUsedColumnList())

        # Checkng for DUPLICATE values
        data.drop_duplicates(keep='first', inplace = True)

        # Drop NA's (rows with missing values)
        data.dropna(inplace=True,axis=1)

        #adds the last time value as the value for time in all rows (when averaging the time value will be the last recorded time value)
        last_value = data['time'].iat[-1]
        data["time"] = last_value

        # encoding rone_reason,  goal reached = 1, timeout = 0 
        contains_goal_reached = data['done_reason'].str.contains('\[\'goal reached\'\]').any()
        if(contains_goal_reached):
            data['done_reason'] = 1
        else:
            data['done_reason'] = 0

        # turn boolean values to numberical ones (true = 1, false = 0)
        data["collision"] = data["collision"].astype(int)

        # in case a collision has accured done_reason will be set to 0 and collision will be set to 1 for the whole episode
        exists_collision = 1 in set(data.collision)
        if(exists_collision):
            data['done_reason'] = 0
            data["collision"] = 1

        
        # creating new columns for the planner teb, dwa, mpc, rlca, arena, rosnav
        data["teb"] = 0
        data["dwa"] = 0
        data["mpc"] = 0
        data["rlca"] = 0
        data["arena"] = 0
        data["rosnav"] = 0
        
        # one hot encoding local_planner"
        data[local_planner] = 1
        

        # refactor speed_dynamic_obs
        dataFrame_Speed_dynamic_obs = RecordedAverage.extractArray(data, "speed_dynamic_obs")
        # Dropping speed_dynamic_obs out of Data Frame "data"
        data=data.drop(columns="speed_dynamic_obs")

        # refactor size_dynamic_obs
        dataFrame_size_dynamic_obs = RecordedAverage.extractArray(data, "size_dynamic_obs")
        # Droppings ize_dynamic_obs  out of Data Frame "data"
        data=data.drop(columns="size_dynamic_obs")
        
        # refactor size_static_obs
        dataFrame_size_static_obs = RecordedAverage.extractArray(data, "size_static_obs")
        # Dropping size_static_obs  out of Data Frame "data"
        data=data.drop(columns="size_static_obs")
        
        # Merges all Data Frames togehter
        data = pd.concat([data, dataFrame_Speed_dynamic_obs, dataFrame_size_dynamic_obs, dataFrame_size_static_obs], axis=1)

        # Creates dataFrame with all the averages
        dataFrame_averages = pd.DataFrame(data.mean().to_dict(),index=[data.index.values[-1]])

        # iterated through the columns of dataFrame_averages and sets the column of averageDataFrameExtended to the same value
        for column in dataFrame_averages:
            averageDataFrameExtended[column] = dataFrame_averages[column]

        # sets all empty / NaN columns to 0
        averageDataFrameExtended = averageDataFrameExtended.fillna(0)

        return averageDataFrameExtended
    
    def checkCSV(dataFrame):

        if(dataFrame.empty):
            print("CSV is empty")
            return False

        csvColumnNames=dataFrame.columns.values.tolist()
        expectedColumnNames = CSVFormat.returnOriginalCSVColumnList()

        if(set(csvColumnNames) != set(expectedColumnNames)):
            print("CSV has wrong format")
            return False
        return True    

if __name__ == "__main__":

    dirname = os.path.dirname(__file__)
    image_path = os.path.join(dirname, "../../../../arena-rosnav/simulator_setup/maps") 
    csv_path = os.path.join(dirname, "../project_recordings")

    parser = ArgumentParser()
    parser.add_argument(
        "--image_path",
        action="store",
        dest="image_path",
        default=image_path,
        help="path to the floor plan of your world. Usually in .pgm format",
        required=False,
    )
    parser.add_argument(
        "--csv_path",
        action="store",
        dest="csv_path",
        default=csv_path,
        help="path to the csv file you want to use as input",
        required=False,
    )
    parser.add_argument(
        "--run_wcs",
        action="store",
        dest="run_wcs",
        default=True,
        help="indicates if the world complexity script should be executed",
        required=False,
    )

    args = parser.parse_args()
    
    RecordedAverage.createAverages(args)
