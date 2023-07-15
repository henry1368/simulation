

#ifndef EVENT_H_
#define EVENT_H_            //时间的定义和相关函数


class Event {
public:
	Event();
	virtual ~Event();

	void
	SetTimeStamp (double time);
	double
	GetTimeStamp () const;
	void
	SetUID (int uid);
	int
	GetUID () const;
	char *
	GetAttribution() const;
	void
	SetAttribution(char* attribution);
	
	virtual void RunEvent () = 0;

private:
	char*  m_attribution;
	double m_timeStamp;
	int m_uid;
};

#endif
