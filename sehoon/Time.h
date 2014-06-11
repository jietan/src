#ifndef SEHOON_TIME_MANAGEMENT_H
#define SEHOON_TIME_MANAGEMENT_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace sehoon {
    namespace time {
        static boost::posix_time::ptime now();
        static double elapsed(const boost::posix_time::ptime& begin_time);
        static double elapsed(const boost::posix_time::ptime& end_time,
                              const boost::posix_time::ptime& begin_time);
        static void wait(double seconds);
        class ProfilerEntry;
        class Profiler;
        class scoped_profile;
        
        static boost::posix_time::ptime now() {
            return boost::posix_time::microsec_clock::local_time();
        }

        static double elapsed(const boost::posix_time::ptime& begin_time) {
            return elapsed(now(), begin_time);
        }

        static double elapsed(const boost::posix_time::ptime& end_time,
                              const boost::posix_time::ptime& begin_time) {
            static const double MICRO_PER_SEC = 
                static_cast<double>(
                    boost::posix_time::time_duration(
                        boost::posix_time::seconds(1)
                        ).total_microseconds()
                    ); 
            
            double elapsed = static_cast<double>(
                (end_time - begin_time).total_microseconds() / MICRO_PER_SEC );
            return elapsed;
        }
  
        static void wait(double seconds) {
            boost::posix_time::ptime start_time = now();
            while(elapsed(start_time) < seconds);
        }



        class ProfilerEntry {
        public:
            ProfilerEntry(const char* const _name)
                : name(_name) {
                count = 0;
                sum = 0.0;
                sumsq = 0.0;
            }

            void start() {
                lastTime = now();
            }

            void stop() {
                double t = elapsed( lastTime );
                ++count;
                sum += t;
                sumsq += (t * t);
            }

            std::string getName() const {
                return name;
            }

            int getCount() const {
                return count;
            }
            
            double getSum() const {
                return sum;
            }

            double getAverage() const {
                return sum / static_cast<double>(count);
            }
            
        private:
            std::string name;
            boost::posix_time::ptime lastTime;
            int count;
            double sum;
            double sumsq;
        };

        class Profiler {
        public:
            Profiler() : defaultEntry(NULL) {
                constructedTime = now();
            };

            virtual ~Profiler() {
                BOOST_FOREACH(ProfilerEntry* e, entries) {
                    delete e;
                }
                entries.clear();
            }

            ProfilerEntry* createEntry(const char* const name) {
                ProfilerEntry* e = new ProfilerEntry(name);
                entries.push_back(e);
                return e;
            }

            std::string toString(ProfilerEntry* e) const {
                std::stringstream sout("");
                sout << "[" << e->getName() << "] : "
                     << "cnt " << e->getCount() << " "
                     << "total " << e->getSum() << "s. "
                     << "avg " << e->getAverage() << "s. "
                     << "ratio " << 100.0 * e->getSum() / getTotalElapsed() << "%. "
                    ;
                    
                return sout.str();
            }

            std::string toString() const {
                std::stringstream sout("");

                sout << endl;
                sout << "======================================================" << endl;
                sout << " Profiler" << endl;
                sout << "======================================================" << endl;
                BOOST_FOREACH(ProfilerEntry* e, entries) {
                    sout << toString(e) << endl;
                }
                sout << "======================================================" << endl;
                sout << " " << entries.size() << " Entries" << endl;
                sout << " total " << getTotalElapsed() << endl;
                sout << "======================================================" << endl;
                sout << endl;
                
                return sout.str();
            }

            void print() {
                cout << this->toString() << endl;
            }

            double getTotalElapsed() const {
                if (defaultEntry) {
                    return defaultEntry->getSum();
                }
                return elapsed(constructedTime);
            }

            void setDefaultEntry(ProfilerEntry* entry) {
                defaultEntry = entry;
            }

        private:
            boost::posix_time::ptime constructedTime;
            std::vector<ProfilerEntry*> entries;
            ProfilerEntry* defaultEntry;
        };


        class scoped_profile {
        public:
            scoped_profile(ProfilerEntry* _entry)
                : entry(_entry) {
                if (entry) {
                    entry->start();
                }
            }

            ~scoped_profile() {
                if (entry) {
                    entry->stop();
                }
            }
        private:
            ProfilerEntry* entry;
        };
        
    } // namespace time
} // namespace sehoon {


#endif // #ifndef SEHOON_TIME_MANAGEMENT_H

