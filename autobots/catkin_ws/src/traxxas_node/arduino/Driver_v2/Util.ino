/**
 * Computes the time difference between time1 and time2.  It assumes
 * that time1 is before time2.  Typically, time1 and time2 are obtained
 * using millis(), which is the number of milliseconds that have elapsed
 * since the Aruino powered up.  The max value returned by this timer
 * is 4,294,967,295 (2^32 - 1).  Thus, the timer overflows once every
 * 4,294,967,295 / 1000 / 60 / 60 / 24 = 49.7 days.  This method should
 * gracefully handle overflow situations.
 */
unsigned long calcTimeDiff(unsigned long time1, unsigned long time2) {
  if (time1 > time2) {
    unsigned long maxTime = 0xFFFFFFFF;
    return maxTime - time2 + time1;
  } else
    return time2 - time1;
}
