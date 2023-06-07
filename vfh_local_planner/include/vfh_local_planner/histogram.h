namespace vfh_local_planner {
    template<typename T>
    class Histogram {
    private:
        std::vector<T> histogram_;
        unsigned int number_of_sector_;
    public:
        Histogram(unsigned int number_of_sector, T default_number = 0){
            histogram_.assign(number_of_sector, default_number);
            number_of_sector_ = number_of_sector;
        }

        T& operator[](int idx){
            if(idx < 0)
                idx = number_of_sector_ - (idx%number_of_sector_);

            return histogram_[idx%number_of_sector_];
        }

        unsigned int getNumberOfSector(){
            return number_of_sector_;
        }
    };
}