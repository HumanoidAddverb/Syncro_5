#include "addverb_cobot_hardware/cobot_executor.h"

namespace addverb_cobot
{
    /**
     * @brief shutdown and join thread
     */
    CobotExecutor::~CobotExecutor()
    {
        std::cout<<"dtor cobot_executore\n";
        shutdown_();
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
    }

    /**
     * @brief setup executor and waiting for spinning to start
     * @return
     */
    bool CobotExecutor::setup()
    {
        spin_thread_ = std::thread(&CobotExecutor::run_, this);

        for (int count = 0; count < count_limit_; count++)
        {
            if (this->spinning)
            {
                return true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        shutdown_();
        return false;
    }

    /**
     * @brief run spin() insid the thread continously
     * @return
     */
    void CobotExecutor::run_()
    {
        spin();
    }

    /**
     * @brief shutdown executor
     * @return
     */
    void CobotExecutor::shutdown_()
    {
        if (this->spinning)
        {
            this->cancel();
        }
    }
}