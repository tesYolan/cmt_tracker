This the a wrapper code for CMT. 

First make sure the CMT library is installed

    steps in the CMT installation prodedure. 

Second 

    roslaunch cmt_tracker cmt_tracker.launch
    
Third: to visualize the results of the cmt tracker we use the rqt plugin developed for the application. 

    rqt -s rqt_tracker_view/tracker_plugin

###TODOs

* Enable dynamic reconfigure for the threshold variables
* Enbale pi vision visualization for the application .
* Enable better rules for the automatic rules. 

