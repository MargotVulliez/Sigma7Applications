#ifndef WLOGGER_H
#define WLOGGER_H

#include <fstream>
#include <unistd.h>
#include <chrono>
using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::microseconds;
#include <Eigen/Dense>
#include <thread>
#include <vector>

namespace Logging {

// Log formatter
// TODO: allow user defined log formatter
Eigen::IOFormat logVecFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ");

// interface class
class IEigenVector {
public:
	virtual void print(std::ostream& os) = 0;
};

// template class to encapsulate matrix pointer
template <typename Derived>
class EigenVector: public IEigenVector {
public:
	// ctor
	EigenVector(Eigen::MatrixBase <Derived>* data) { _data = data;}
	// data to encapsulate
	Eigen::MatrixBase <Derived> * _data;
	// implementation of pure virtual function in interface class
	void print (std::ostream& os) {
		 os << _data->transpose().format(logVecFmt);
	}
};

// Logger class
class Logger {
public:
	// ctor
	Logger(long interval, std::string fname, const double realtime_scaling_factor = 1.0)
	: _log_interval_(interval)
	{
		// create log file
		_logfile.open(fname, std::ios::out);
		_logfile << "timestamp, ";
		_realtime_scaling_factor = realtime_scaling_factor;
	}

	// add Eigen vector type variable to watch
	template <typename Derived>
	bool addVectorToLog (Eigen::MatrixBase <Derived>* var, const std::string& var_name = "") {
		if (_f_is_logging) {
			return false;
		}
		auto e = new EigenVector<Derived>(var);
		_vars_to_log.push_back(dynamic_cast<IEigenVector* >(e));
		// for (uint i = 0; i < var->size(); i++) {
		// 	if (!var_name.empty()) {
		// 		_logfile << var_name << "_" << i << ", ";
		// 	} else {
		// 		_logfile << "var" << _vars_to_log.size() << "_" << i << ", ";
		// 	}
		// }
		if (!var_name.empty()) {
			_logfile << var_name << "[" << var->size() << "], ";
		} else {
			_logfile << "var[" << _vars_to_log.size() << "], ";
		}
		return true;
	}

	// start logging
	bool start() {
		// save start time
		_t_start = system_clock::now();

		// set logging to true
		_f_is_logging = true;

		// complete header line
		_logfile << "\n";

		// start logging thread by move assignment
		_log_thread = std::thread{&Logger::logWorker, this};

		return true;
	}

	void stop() {
		// set logging false
		_f_is_logging = false;

		// join thread
		_log_thread.join();

		// close file
		_logfile.close();
	}

	// vector of pointers to encapsulated Eigen vector objects that are registered with 
	// the logger
	std::vector<IEigenVector *> _vars_to_log;

	// state
	bool _f_is_logging;

	// start time
	system_clock::time_point _t_start;

	// log interval in microseconds
	long _log_interval_;

	// log file
	std::fstream _logfile;

	// thread
	std::thread _log_thread;

	// realtime scaling factor
	double _realtime_scaling_factor;

private:
	// thread function for logging. Note that we are not using mutexes here, no there might be weirdness
	void logWorker () {
		system_clock::time_point curr_time;
		system_clock::time_point last_time = system_clock::now();
		while (_f_is_logging) {
			usleep(_log_interval_/2);
			curr_time = system_clock::now();
			auto time_diff = std::chrono::duration_cast<microseconds>(curr_time - last_time);
			if (_log_interval_ > 0 && time_diff >= microseconds(static_cast<uint>(_log_interval_))) {
				microseconds t_elapsed = std::chrono::duration_cast<microseconds>(curr_time - _t_start);
				_logfile << t_elapsed.count() * _realtime_scaling_factor;
				for (auto iter: _vars_to_log) {
					_logfile << ", ";
					iter->print(_logfile);
				}
				_logfile << "\n";
				last_time = curr_time;
			}
		}
	}

	// hide default constructor
	Logger () {

	}
};

}

#endif //WLOGGER_H