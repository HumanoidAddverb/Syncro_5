/**
 * @file data_store.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Data Store - responsible for storing data and retriveing stored data
 * maintains the set of recorded files
 * records data after opening a bucket identified by a unique bucket ID
 * @version 0.1
 * @date 2024-07-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef DATA_STORE_H_
#define DATA_STORE_H_

#include <string>
#include <memory>
#include <fstream>
#include <algorithm>
#include "ui_data_types.h"
#include "error_manager.h"
#include "data_adapter.h"

class DataStore
{
public:
    /// @brief ctor
    DataStore()
    {
        // load the stored bucket IDs upon store creation
        load_();
        std::cout << "DataStore()\n";
    };

    /// @brief dtor
    ~DataStore()
    {
        // close the last opened bucket
        closeBucket();
        // update the store with the labels added in this run of the application
        updateStore_();

        std::cout << "~DataStore()\n";
    };

    /// @brief setup the store with error manager
    /// @param
    /// @return
    bool setupStore(std::shared_ptr<ErrorManagerInterface> &);

    /// @brief initialise storage of given bucket
    /// @param label : the ID of the bucket
    /// @return true/false : true if ID does not exist in the database
    bool initBucket(const std::string& label);

    /// @brief store data by reading current joint position as std::vector
    /// @param
    /// @return
    bool store(const std::vector<double>&, const double &time);

    /// @brief store data by reading current joint position from the feedback as recd from the robot
    /// @param
    /// @return
    bool store(const RobotFeedback&, const double &time);

    /// @brief store data by reading the list of target positions from the target_pos_seq field in the ControllConfig
    /// @param
    /// @return
    bool store(const ControllerConfig&);

    /// @brief store data point by point
    /// @param
    /// @return
    bool store(const AdditionalInput&);

    /// @brief get list of all buckets in record
    /// @return
    std::vector<std::string> getStoredID() const;

    /// @brief close the currently open storage object
    bool closeBucket();

    /// @brief fetch if the requested bucket is in store
    /// @return true - if the bucket with this ID exists
    bool isIDInStore(const std::string &label);

    /// @brief get the stored dataset, upon specifying the bucket ID
    /// @param label
    /// @return
    bool retreiveData(const std::string &label, RecordedDataSet &);

    /// @brief get the stored dataset, upon specifying the bucket ID
    /// @param label
    /// @return
    bool retreiveData(const std::string &label, std::vector<std::vector<double>> &jpos, 
                        std::vector<double> &time);

    /// @brief return the open status of the bucket
    /// @return
    bool isBucketOpen_() const;

    /// @brief return the number of entries in the currently open bucket
    /// @return
    int getNumOfEntries() const;

    /// @brief return the entry number of paricular label
    /// @return
    int getLabelId(const std::string &);

    /// @brief return the label of particular id 
    /// @return
    std::string getIdLabel(const int);

    /// @brief load store 
    /// @return
    void loadStore();

private:
    /// @brief list of recorded bucket IDs - will be populated
    std::vector<std::string> bucket_list_;

    /// @brief bucket in which data is to be stored
    std::ofstream bucket_;

    /// @brief error manager
    std::shared_ptr<ErrorManagerInterface> error_man_;

    /// @brief data set to populate using each entry
    RecordedDataSet data_;

    /// @brief adapter to convert from structure to string and vice-versa
    DataAdapter adapter_;

    /// @brief load the IDs of the pre-stored buckets
    void load_();

    /// @brief update the store with the latest bucket ID list
    void updateStore_();

    /// @brief fill the bucket with the data before closing the lid on the bucket :)
    bool fillBucket_();

    /// @brief get back entry in the standard format from the input provided
    /// @param
    /// @param
    /// @return
    RecordedDataEntry getEntry_(const std::vector<double>&,const double&);

    /// @brief update entry in the data set
    /// @param
    /// @param
    void updateEntry_(const std::vector<double>&, const double&);
};

#endif