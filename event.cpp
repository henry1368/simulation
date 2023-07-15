


#include "event.h"

Event::Event()
{}

Event::~Event()
{}

void
Event::SetTimeStamp (double time)
{
  m_timeStamp = time;
}

double
Event::GetTimeStamp () const
{
  return m_timeStamp;
}

void
Event::SetUID (int uid)
{
  m_uid = uid;
}

int
Event::GetUID () const
{
  return m_uid;
}

char *
Event::GetAttribution() const
{
	return m_attribution;
}
void
Event::SetAttribution(char* attribution)
{
	m_attribution = attribution;
} 