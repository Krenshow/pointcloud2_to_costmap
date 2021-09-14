// .h
#include "pointcloud2_to_costmap.h"


//................................................CONFIG......................................................

void PointCloudConverter::read_config(ros::NodeHandle *nh) {
    try
    {
        if (!nh->hasParam(param_name)) {
            throw std::runtime_error("Error! Could not find param \'"+ param_name + "\' in config.yaml.");
        }
        else {
            XmlRpc::XmlRpcValue typeList;
            try {
                bool print_conf = false;
                nh->getParam("/print", print_conf);
                nh->getParam("/types", typeList);
                for (int i = 0; i < typeList.size(); ++i) {
                    Config tmp;
                    tmp.name = static_cast<std::string>(typeList[i]["name"]);
                    tmp.value = static_cast<int>(typeList[i]["value"]);
                    tmp.group = static_cast<std::string>(typeList[i]["group"]);
                    config.push_back(tmp);
                }
                print_config(print_conf);
            }
            catch (const XmlRpc::XmlRpcException &e) {
                std::cout << "Error! XmlRpcException: probably you have to write integer number ('0' or '25')." << std::endl;
            } 
        }
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
    } 
}

void PointCloudConverter::print_config(bool print_conf) {
    if (print_conf) {
        std::cout << "\n______" << std::endl;
        std::cout << "CONFIG\n" << std::endl;
        for (int i = 0; i < config.size(); ++i) {
            std::cout << "name: " << config[i].name << std::endl;
            std::cout << "value: " << config[i].value << std::endl;
            std::cout << "group: " << config[i].group << std::endl;
            std::cout <<  std::endl;
        }
    }
}


//............................................PointCloudType..................................................

// Convert 4 bytes to Float32
float PointCloudType::Convert(unsigned int (&c) [4]) {
    UStuff b;
    for (size_t i = 0; i < 4; ++i) {
            b.c[i] = *(c+i);
    }
    return b.f;
}

// send a group of points classes as PointCloud2 message 
void PointCloudType::send_msg(const sensor_msgs::PointCloud2ConstPtr& msg){
    unsigned int width = 0;

    _msg.data.clear();
    _msg.header = msg->header;
    _msg.height = msg->height;
    _msg.fields = msg->fields;
    _msg.is_bigendian = msg->is_bigendian;
    _msg.point_step = msg->point_step;
    _msg.row_step = msg->row_step;
    _msg.is_dense = msg->is_dense;

    for(int i = 0; i < (msg->data.size()/16); ++i) { 
        unsigned int colori[4];
        float color;
        for (int j = 0; j < 4; ++j) {  
            colori[j] = int(msg->data[16*i+12+j]);
        }
        color = Convert(colori);
        
        for (int j = 0; j < (values.size()); j++) {
            if (color == values[j]){
                for (int k = 0; k < 16; ++k) { 
                    _msg.data.push_back(msg->data[16*i+k]);
                }
                width++;
            }
        }
        _msg.width = width;
    }
    pub.publish(_msg);
}


//..........................................PointCloudConverter...............................................

void PointCloudConverter::Callback(const sensor_msgs::PointCloud2ConstPtr& msg) {    
    for (auto &type : types){
        type.send_msg(msg);
    }
}

void PointCloudConverter::create_types(ros::NodeHandle *nh){
    for (auto &config_ : config){
        bool push = true;
        if (types.size() > 0){
            for (auto &type_ : types){
                if (type_.group == config_.group){
                    push = false;
                }
            }
            if (push){
                PointCloudType tmp = PointCloudType(nh, config_.group);
                types.push_back(tmp);
            }
        }
        else{
            PointCloudType tmp = PointCloudType(nh, config_.group);
            types.push_back(tmp);
        }
    }
    for (auto &config_ : config){
        for (auto &type_ : types){
            if (config_.group == type_.group){
                type_.values.push_back(config_.value);
            }
        }
    }
}

void PointCloudConverter::print_types(){
    std::cout << "\n______" << std::endl;
    std::cout << "GROUPS\n" << std::endl;
    for (auto &type : types){
        std::cout << "type->group: " << type.group << std::endl;
        for (auto &value : type.values){
            std::cout << "value: " << value << std::endl;
        }
        std::cout <<  std::endl; 
    }
}

//.................................................Main.......................................................

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud2_to_costmap");
    ros::NodeHandle nh;
    PointCloudConverter pc = PointCloudConverter(&nh);
    ros::spin();
    return 0;
}