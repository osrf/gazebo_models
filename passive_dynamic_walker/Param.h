#ifndef _GAZEBO_PARAM_H
#define _GAZEBO_PARAM_H

#include <cstdlib>
#include <list>
#include <string>
#include <stdexcept>

namespace gazebo {

// The parameter structure
class Param
{
  public:
    enum ParamType { eUnknown, eInt, eReal };

    // the type of parameter
    ParamType ptype;   

    // number of int parameters 
    int int_start, int_end; 

    // start, step, and end of real range
    double r_start, r_step, r_end; 

    // the name of the parameter
    std::string name;

    // parameter options 
    std::list<std::string> options; 

    // gets the type of parameter (int, real)
    static ParamType get_param_type(const std::string& ptype)
    {
      if (ptype == "int")
        return eInt;
      else if (ptype == "real")
        return eReal;
      else
        return eUnknown;
    }

    // builds the parameter based on the token string
    static Param build_param(const std::list<std::string>& tokens)
    {
      std::list<std::string>::const_iterator token_iter = tokens.begin();

      // create the parameter
      Param p;

      // read the parameter name
      if (token_iter == tokens.end())
        throw std::runtime_error("Bad token information");
      p.name = *token_iter++;

      // set the parameter type
      if (token_iter == tokens.end())
        throw std::runtime_error("Bad token information");
      p.ptype = get_param_type(*token_iter++);
    
      // if the parameter type is an int, read in the length of the int
      switch (p.ptype)
      {
        case eInt: 
          if (token_iter == tokens.end())
            throw std::runtime_error("Bad token information");
          p.int_start = std::atoi(token_iter->c_str());
          token_iter++;
          if (token_iter == tokens.end())
            throw std::runtime_error("Bad token information");
          p.int_end = std::atoi(token_iter->c_str());
          token_iter++;
          break;

        case eReal:
          if (token_iter == tokens.end())
            throw std::runtime_error("Bad token information");
          p.r_start = std::atof(token_iter->c_str());
          token_iter++;
          if (token_iter == tokens.end())
            throw std::runtime_error("Bad token information");
          p.r_step = std::atof(token_iter->c_str());
          token_iter++;
          if (token_iter == tokens.end())
            throw std::runtime_error("Bad token information");
          p.r_end = std::atof(token_iter->c_str());
          token_iter++;
          break;

        default:
          throw std::runtime_error("Unexpected token type");
      }

      // remaining are options
      while (token_iter != tokens.end())
        p.options.push_back(*token_iter++);

      return p;
    }
};

inline std::ostream& operator<<(std::ostream& out, const Param& p)
{
  out << p.name << " ";
  if (p.ptype == Param::eInt)
  {
    out << "int ";
    out << p.int_start << " " << p.int_end;
  }
  else if (p.ptype == Param::eReal)
  {
    out << "real ";
    out << p.r_start << " " << p.r_step << " " << p.r_end;
  }
  else
    out << "unknown";

  for (std::list<std::string>::const_iterator i = p.options.begin(); i != p.options.end(); i++)
    out << " " << *i;
  out << std::endl; 
}

} // end namespace

#endif

