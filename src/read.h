#ifndef READ_H
#define READ_H
 

 std::vector<std::vector<double>> GetPoses(const std::string & file_name);
 void ReadDistortionParameters(const std::string &file, double &k1, double &k2);
 void ReadCameraMatrix(const std::string &file, Eigen::MatrixXd &K);
 int GetNumberofImages(const std::string & path, const std::string & type);

 
#endif