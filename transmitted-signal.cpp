#include "transmitted-signal.h"

#include <iostream>

TransmittedSignal::TransmittedSignal()
{}

TransmittedSignal::~TransmittedSignal()
{}

void
TransmittedSignal::SetValues (std::vector<double> values)
{
  m_values = values;
}

std::vector<double>
TransmittedSignal::Getvalues ()
{
  return m_values;
}

TransmittedSignal*
TransmittedSignal::Copy ()
{
  TransmittedSignal* txSignal = new TransmittedSignal ();
  txSignal->SetValues (Getvalues ());
  return txSignal;
}
