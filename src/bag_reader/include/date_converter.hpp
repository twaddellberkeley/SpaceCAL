#include <iostream>

#define SECS_IN_YEAR 31536000
#define SECS_IN_DAY 86400
#define SECS_IN_HOUR 3600
#define SECS_IN_MIN 60
#define SECS_IN_LEAP_YEAR SECS_IN_YEAR + SECS_IN_DAY
#define NUM_OF_LEAP_YEARS 13
#define FOUR_YEAR_INCREMENT_SEC 126230400UL
#define MAX_DATE_SEC 1656816143UL // 347155200UL
#define MILLI_SEC 1000ULL
#define MICRO_SEC 1000000ULL
#define NANO_SEC 1000000000ULL
#define PACIFIC_TIME_SEC -(SECS_IN_HOUR * 8)
#define PACIFIC_TIME_NANO -(SECS_IN_HOUR * NANO_SEC * 8)

class date_to_str
{
public:
    std::string precition_;
    unsigned long int sec_time_;
    unsigned long int milli_time_;
    unsigned long int micro_time_;
    unsigned long int nano_time_;
    std::string year_str_;
    int year_;
    std::string month_str_;
    int month_;
    int day_;
    int hour_;
    int min_;
    unsigned int sec_;
    unsigned int milli_sec_;
    unsigned int micro_sec_;
    unsigned int nano_sec_;

    bool is_leap_year_;
    int num_of_leap_years_;

    date_to_str(unsigned long int time, std::string units);

    // Constructs a date from number of days since epoch
    void construct_date(int days);

    // Construct time from time left
    void construct_time(unsigned int days);

    // Updates time to new time
    void update_date(unsigned long int time, std::string units);

    void update_precition(unsigned long int time, std::string units);

    // Resets all members to epoch time
    void reset_date();

    std::string str_date();
};
