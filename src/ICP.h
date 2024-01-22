void ICP(const std::vector<Eigen::Vector3f>& p1, const std::vector<Eigen::Vector3f>& p2,
         Eigen::Matrix3f& R_12, Eigen::Vector3f& t_12);
void ReadPcd(std::string filename, std::vector<Eigen::Vector3f>& p1);