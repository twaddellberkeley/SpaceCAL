#include <iostream>
#include "date_converter.hpp"

date_to_str::date_to_str(unsigned long int time, std::string units)
    : year_(1970), month_(1), day_(1), hour_(0), min_(0),
      sec_(0), milli_sec_(0), micro_sec_(0), nano_sec_(0),
      is_leap_year_(false), num_of_leap_years_(0)
{
    date_to_str::update_precition(time, units);
    int days = sec_time_ / SECS_IN_DAY;
    date_to_str::construct_date(days);
    date_to_str::construct_time(days);
}

// Constructs a date from number of days since epoch
void date_to_str::construct_date(int days)
{
    // using namespace std;
    int month_days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (days < 0)
    {
        // Correct starting time and first leap year (begining of year 68 and go backwards);
    }
    else if (days > (365 * 3))
    {                                // check if we are whithin the first leap year
                                     // find the closest leap year
        days = days - (365 * 3 + 1); // if we are here we can subtract the first leap year
        year_ = 1973;                // we must at least be at the year after the first leap year
        num_of_leap_years_++;        // we have at least one leap year
        while (days > (365 * 4))
        {                          // stop when we get close to the last leap year
            days -= (365 * 4 + 1); // subtract four years of days passed
            year_ += 4;            // add four years to our year
            num_of_leap_years_++;  // increase leap year by one
        }
    }
    // Then -> find year
    int counter = 1;
    if (year_ == 1970)
        counter = 2;
    while (days >= 365 && counter < 4)
    { // while days is greater then a year and not a leap year
        days -= 365;
        // cout << "days: " << days << "  year: " << year_ << " counter: " << counter << endl;
        year_++;
        counter++;
    }
    if (counter == 4)
    {
        is_leap_year_ = true; // if couter reached 4 then we are in a leap year
        num_of_leap_years_++;
    }

    // Find the month
    int index = 0;
    if (is_leap_year_)
        month_days[1] = 29;
    while (index < 12 && (days + 1) > month_days[index])
    {
        days -= month_days[index];
        // cout << "days: " << days << " month " << index + 1 << endl;
        month_++;
        index++;
    }
    // finally update day
    day_ += days;
}

// Construct time from time left
void date_to_str::construct_time(unsigned int days)
{
    unsigned long int used_sec = days * SECS_IN_DAY;
    unsigned int sec_left = sec_time_ - used_sec;
    hour_ = sec_left / SECS_IN_HOUR;
    sec_left = sec_left % SECS_IN_HOUR;
    min_ = sec_left / SECS_IN_MIN;
    sec_left = sec_left % SECS_IN_MIN;
    sec_ = sec_left;
}

// Updates time to new time
void date_to_str::update_date(unsigned long int time, std::string units)
{
    reset_date();
    update_precition(time, units);
    int days = sec_time_ / SECS_IN_DAY;
    construct_date(days);
}

void date_to_str::update_precition(unsigned long int time, std::string units)
{
    if (units == "sec")
    {
        time = time + PACIFIC_TIME_SEC;
        sec_time_ = time;
    }
    else if (units == "milli")
    {
        milli_time_ = time;
        sec_time_ = time / MILLI_SEC;
        milli_sec_ = time % MILLI_SEC;
    }
    else if (units == "micro")
    {
        micro_time_ = time;
        sec_time_ = time / MICRO_SEC;
        micro_sec_ = time % MICRO_SEC;
    }
    else if (units == "nano")
    {
        time = time + PACIFIC_TIME_NANO;
        nano_time_ = time;
        sec_time_ = time / NANO_SEC;
        nano_sec_ = time % NANO_SEC;
    }
    precition_ = units;
}

// Resets all members to epoch time
void date_to_str::reset_date()
{
    nano_time_ = sec_time_ = hour_ = min_ = sec_ = num_of_leap_years_ = 0;
    milli_sec_ = micro_sec_ = nano_sec_ = 0;
    year_str_ = month_str_ = "";
    year_ = 1970;
    month_ = 1;
    day_ = 1;
    is_leap_year_ = false;
}

std::string date_to_str::str_date()
{
    std::string date = "";

    // Get month
    switch (month_)
    {
    case 1:
        /* code */
        month_str_ = "January";
        break;
    case 2:
        month_str_ = "February";
        break;
    case 3:
        month_str_ = "March";

        break;
    case 4:
        month_str_ = "April";

        break;
    case 5:
        month_str_ = "May";

        break;
    case 6:
        month_str_ = "June";

        break;
    case 7:
        month_str_ = "July";

        break;
    case 8:
        month_str_ = "August";

        break;
    case 9:
        month_str_ = "September";

        break;
    case 10:
        month_str_ = "October";

        break;
    case 11:
        month_str_ = "November";

        break;
    case 12:
        month_str_ = "December";

        break;

    default:
        break;
    }

    date = std::to_string(year_) + "-" + month_str_ + "-" + std::to_string(day_) + "-" +
           std::to_string(hour_) + ":" + std::to_string(min_) + ":" + std::to_string(sec_) + "." +
           std::to_string(nano_sec_);

    return date;
}