#include <knowledge_representation/convenience.h>
#include <string>

namespace knowledge_rep
{
LongTermMemoryConduit getDefaultLTMC()
{
  std::string db_name = "knowledge_base";
  std::string db_hostname = "localhost";
  // The expectation is that you won't put secrets directly in your code or out in environment variables,
  // so LTMC implementations will either look for passwords in some dotfile or rely on being on an allow list
  // with the backend.
  if (const char* env_db_name = std::getenv("KNOWLEDGE_REP_DB_NAME"))
  {
    db_name = env_db_name;
  }
  if (const char* env_hostname = std::getenv("KNOWLEDGE_REP_DB_HOSTNAME"))
  {
    db_name = env_hostname;
  }
  return LongTermMemoryConduit(db_name, db_hostname);
}

}  // namespace knowledge_rep
