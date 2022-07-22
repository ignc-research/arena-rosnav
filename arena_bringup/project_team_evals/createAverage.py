import pandas as pd
import glob
import sys
import warnings
import pathlib as pl
warnings.simplefilter(action='ignore', category=FutureWarning)

class RecordedAverage:

    episodeAverages = pd.DataFrame()


    def initCSVDf():
        lst = list(range(0,20))
        append_str = "speed_dynamic_obs_"
        speed_dynamic_obs_column = [append_str + str(sub) for sub in lst]
        append_str = "size_dynamic_obs_"
        size_dynamic_obs_column = [append_str + str(sub) for sub in lst]
        append_str = "size_static_obs_"
        size_static_obs_column = [append_str + str(sub) for sub in lst]

        robotList = ["burger", "jackal","ridgeback","agvota","rto","rto_real"]
        plannerList = ["teb","dwa","mpc","rlca","arena","rosnav"]

        columnList =["time","done_reason","collision", "robot_radius", "robot_max_speed","number_dynamic_obs","number_static_obs"]
        columnList = columnList + robotList + plannerList + size_static_obs_column + size_dynamic_obs_column + speed_dynamic_obs_column
        df = pd.DataFrame(columns=columnList)
        return df

    def createAverages():
        commandLineArguments = sys.argv
        pathToCSV = commandLineArguments[1]
        outputPath = "{}/CSVaverages/".format(pathToCSV)
        print("")
        print(outputPath)
        print("")
        path = pl.Path(outputPath)
        path.mkdir(parents=True)


        if(len(commandLineArguments) != 2):
            print("Wrong command line arguments. Please specify the path to the directory containing the CSV files")
        print("path to the directory containing the CSV files:", pathToCSV)    

        for fileName in glob.glob('{}*.csv'.format(pathToCSV)):
            print("Currently working on:",fileName)
            RecordedAverage.episodeAverages = RecordedAverage.initCSVDf()

            data = pd.read_csv(fileName)

            if(not(RecordedAverage.checkCSV(data))):
                continue

            RecordedAverage.createEpisodeAverage(data)
        
            csvAverage = RecordedAverage.calculateCSVAverage(RecordedAverage.episodeAverages)

            csvFilename = "{}/averages.csv".format(outputPath)
            with open(csvFilename, 'a') as f:
                csvAverage.to_csv(f, mode='a', header=f.tell()==0, index=False)
            

    def calculateCSVAverage(dataFrame):
        average = pd.DataFrame(dataFrame.mean().to_dict(),index=[dataFrame.index.values[-1]])
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
    
        dataFrame_extractedColumn = dataFrame_extractedColumn.add_prefix(columnName+"_")
        # Changing type String to type float for every column
        
        patternNew  ='|'.join(["\[","\]","\'"])
        
        for col in dataFrame_extractedColumn:
            dataFrame_extractedColumn[col] = dataFrame_extractedColumn[col].str.replace(patternNew,"")
            dataFrame_extractedColumn[col] = dataFrame_extractedColumn[col].astype(float)
        return dataFrame_extractedColumn        
        

    def calculateEpisodeAverage(data):

        averageDataFrameExtended = RecordedAverage.initCSVDf()

        # Drop unnecessary columns
        data = data.drop(columns=["laser_scan",
                   "robot_lin_vel_x",
                  "robot_lin_vel_y",
                  "robot_ang_vel",
                   "robot_orientation",
                   "robot_pos_x",
                   "robot_pos_y",
                   "action",
                   "episode",
                   "form_dynamic_obs",
                   "form_static_obs",
                   "map"
                  ])

        # Checkng for DUPLICATE values
        data.drop_duplicates(keep='first', inplace = True)

        # Drop NA's (rows with missing values)
        data.dropna(inplace=True,axis=1)

        # trune boolean values to numberical ones (true = 1, false = 0)
        data["collision"] = data["collision"].astype(int)

        #adds the last time value as the value for time in all rows (when averaging the time value will be the last recorded time value)
        last_value = data['time'].iat[-1]
        data["time"] = last_value

        # encoding rone_reason,  goal reached = 1, timeout = 0 
        boolean_finding = data['done_reason'].str.contains('\[\'goal reached\'\]').any()
        if(boolean_finding):
            data['done_reason'] = 1
        else:
            data['done_reason'] = 0

        # creating new columns for the robot versions
        data["burger"] = 0
        data["jackal"] = 0
        data["ridgeback"] = 0
        data["agvota"] = 0
        data["rto"] = 0
        data["rto_real"] = 0

        # one hot encoding in case robot is specified robot
        robotModel = data["robot_model"][0]
        data[robotModel] = 1
        data = data.drop(columns="robot_model")

        # creating new columns for the planner teb, dwa, mpc, rlca, arena, rosnav
        data["teb"] = 0
        data["dwa"] = 0
        data["mpc"] = 0
        data["rlca"] = 0
        data["arena"] = 0
        data["rosnav"] = 0
        
        # one hot encoding local_planner"
        local_planner = data["local_planner"][0]
        data[local_planner] = 1
        data = data.drop(columns="local_planner")

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

        # for col in dataFrame_extractedColumn:
        for column in dataFrame_averages:
            averageDataFrameExtended[column] = dataFrame_averages[column]

        averageDataFrameExtended = averageDataFrameExtended.fillna(0)

        return averageDataFrameExtended
    
    def checkCSV(dataFrame):

        if(dataFrame.empty):
            print("CSV is empty")
            return False

        dataFrame=dataFrame.drop(columns=["laser_scan",
                   "robot_lin_vel_x",
                  "robot_lin_vel_y",
                  "robot_ang_vel",
                   "robot_orientation",
                   "robot_pos_x",
                   "robot_pos_y",
                   "action",
                   "episode",
                   "form_dynamic_obs",
                   "form_static_obs",
                   "map"
                  ])

        csvColumnNames=dataFrame.columns.values.tolist()
        expectedColumnNames=RecordedAverage.initCSVDf().columns.values.tolist()
        originalColNames=["local_planner","robot_model","size_dynamic_obs","size_dynamic_obs","speed_dynamic_obs","size_static_obs"]
        expectedColumnNames = originalColNames + expectedColumnNames
        
        if not(set(csvColumnNames) <= set(expectedColumnNames)):
            print("CSV has wrong format")
            return False
        return True    

if __name__ == "__main__":
    RecordedAverage.createAverages()

