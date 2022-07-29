#include <deque>

typedef bool (*monitor_condition)(std::deque<double>::const_iterator it, double param);

int initMonitor(int len, bool once, monitor_condition cond, double cond_param, int history_len);
void updateMonitor(int id, double val);
bool checkMonitor(int id);

int initMonitorLowerBound(int len, double lower_bound);
int initMonitorUpperBound(int len, double upper_bound);
int initMonitorAboveThreshold(int len, double threshold);
int initMonitorBelowThreshold(int len, double threshold);
