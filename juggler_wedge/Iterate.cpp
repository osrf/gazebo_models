#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <vector>
#include "Param.h"
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

enum ParamType { eUnknown, eInt, eReal };

using std::cerr;
using std::endl;

/// Converts an unsigned long long to a binary vector
void to_binary(unsigned long long i, std::vector<bool>& bin)
{
  // compute highest representable integer using boolean array
  unsigned n = bin.size();
  unsigned long long n2 = 1;
  for (unsigned j=n-1; j > 0; j--)
    n2 *= 2;

  // setup the binary number
  for (unsigned j=0; j< n; j++)
  {
    bin[j] = (i / n2 > 0);
    unsigned long long rem = i % n2;
    i = rem;
    n2 /= 2;
  }
} 

// splits a list of parameters into multiple processes
void split_params(const std::list<gazebo::Param>& params, std::vector<std::list<gazebo::Param> >& split_params, unsigned nfiles)
{
  std::vector<bool> bin;

  // compute the number of bits necessary to represent the array
  unsigned bits = 0;
  unsigned N = nfiles;
  while (N >>= 1) ++bits;

  // resize the binary array
  if (bits == 0)
  {
    split_params.clear();
    split_params.push_back(params);
  }
  else
    bin.resize(bits);

  // iterate over the number of splits
  for (unsigned i=0; i< nfiles; i++)
  {
    // get the binary representation of the split
    to_binary((unsigned long long) i, bin);

    // compute and save the split
    split_params.push_back(params);
    std::list<gazebo::Param>& split = split_params.back();
    std::list<gazebo::Param>::iterator split_iter = split.begin();
    for (unsigned j=0; j< bits; j++, split_iter++)
    {
      bool value = bin[j];
      gazebo::Param& p = *split_iter;
      if (p.ptype == gazebo::Param::eInt)
      {
        if (bin[j])
          p.int_start = (p.int_end + p.int_start)/2;
        else
          p.int_end = (p.int_end + p.int_start)/2;
      }
      else
      {
        if (bin[j])
          p.r_start = (p.r_end + p.r_start)*0.5;
        else
          p.r_end = (p.r_end + p.r_start)*0.5;
      }
    }
  }
}

// tokenizes a string
std::list<std::string> tokenize(const std::string& s)
{
  std::list<std::string> l;

  // tokenize the string
  std::istringstream input(s.c_str());
  while (true)
  {
    // get the next
    std::string next;
    input >> next;
    if (!input)
      break;
    l.push_back(next);
  }

  // transform each string to lower case
  BOOST_FOREACH(std::string& s, l)
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);

  return l;
}

int main(int argc, char* argv[])
{
  std::list<gazebo::Param> params;
  std::vector<std::list<gazebo::Param> > process_params;

  if (argc < 2)
  {
    std::cerr << "syntax: Iterate <filename> <processes>" << std::endl;
    return -1;
  }

  // open the file for reading
  std::ifstream in(argv[1]);

  // loop until all lines are looped over
  while (true)
  {
    // read in the line
    std::string input;
    std::getline(in, input);  
    if (!in)
      break;

    // tokenize it
    std::list<std::string> tokens = tokenize(input);

    // setup the parameters
    if (tokens.size() < 4)
    {
      if (!tokens.empty())
        std::cerr << "Iterator: fewer than four tokens in string: " << input << std::endl;
      continue;
    }

    // create the parameter
    params.push_back(gazebo::Param::build_param(tokens));
  }  

  // split the parameters based on the number of processes
  split_params(params, process_params, std::atoi(argv[2]));

  // write the parameters out to files for processing
  for (unsigned i=0; i< process_params.size(); i++)
  {
    std::string fname = std::string(argv[1]) + "." + boost::lexical_cast<std::string>(i);
    std::ofstream out(fname.c_str());
    BOOST_FOREACH(const gazebo::Param& p, process_params[i])
      out << p;
    out.close();
  }

  // close the file
  in.close();
}

