
#include "vision_utils/utils/Translator.h"
//#include "network/NetworkUtils.h"
#include "vision_utils/utils/error.h"
#include "vision_utils/utils/map_utils.h"
#include "vision_utils/utils/find_and_replace.h"
#include "vision_utils/utils/string_web_retrieve.h"
#include "vision_utils/utils/string_casts_stl.h"
#include "vision_utils/utils/string_split.h"

//! here is defined the active translator engine
//#define USE_GOOGLE_API_V1 // deprecated since june 2011
//#define USE_GOOGLE_API_V2 // costly since december 2011
#define USE_MICROSOFT_TRANSLATOR_V2 // aaaargh, microsoft!

#ifdef USE_GOOGLE_API_V2
static const std::string google_api_key =
    "AIzaSyDugyKvZraqZjIh9038_HZZewrVQu1NwQ0";
#endif // USE_GOOGLE_API_V2

#ifdef USE_MICROSOFT_TRANSLATOR_V2
static const std::string application_id =
    "6844AE3580856D2EC7A64C79F55F11AA47CB961B";
#endif // USE_MICROSOFT_TRANSLATOR_V2

std::string Translator::translate(const std::string & sentence_to_translate,
                                  const LanguageId & language_orig,
                                  const LanguageId & language_dest) {

  maggieDebug2("translate(sentence:'%s'' : %s -> %s)",
               sentence_to_translate.c_str(),
               get_language_full_name(language_orig).c_str(),
               get_language_full_name(language_dest).c_str());

  // don't translate if the languages are identical
  if (language_orig == language_dest)
    return sentence_to_translate;

  // prepair the string for http encoding
  std::string sentence_to_translate_url = sentence_to_translate;
  string_utils::find_and_replace(sentence_to_translate_url,
                                " ", "%20");
  string_utils::find_and_replace(sentence_to_translate_url,
                                "\n", "%0A");

  // find the ides
  LanguageMap language_map;
  build_languages_map(language_map);
  // autodetect language if needed
  LanguageId language_orig_detect = language_orig;
  if (language_orig == LANGUAGE_UNKNOWN)
    language_orig_detect = detect_language(sentence_to_translate);
  if (language_orig_detect == LANGUAGE_UNKNOWN) { // detection failed -> quit
    maggiePrint("Impossible to detect the language of '%s'",
                sentence_to_translate.c_str());
    return sentence_to_translate;
  }

  // don't translate if the languages are identical
  if (language_orig_detect == language_dest)
    return sentence_to_translate;

  CountryDomain prefix_orig, prefix_dest;
  if (!get_country_domain_from_language_id
      (language_map, language_orig_detect, prefix_orig)
      || !get_country_domain_from_language_id(language_map, language_dest, prefix_dest))
    return sentence_to_translate;

  // build the URL
  std::ostringstream url;

#ifdef USE_GOOGLE_API_V1
  //https://ajax.googleapis.com/ajax/services/language/translate?v=1.0&q=Hello,%20my%20friend!&langpair=en%7Ces
  url << "https://ajax.googleapis.com/ajax/services/language/translate?v=1.0";
  url << "&q=" << sentence_to_translate_url;
  url << "&langpair=" << prefix_orig << "|" << prefix_dest;
#endif // USE_GOOGLE_API_V1

#ifdef USE_GOOGLE_API_V2
  url << "https://www.googleapis.com/language/translate/v2?";
  url << "key=" << google_api_key;
  url << "&source=" << prefix_orig;
  url << "&target=" << prefix_dest;
  url << "&prettyprint=false";
  url << "&callback=handleResponse";
  url << "&q=" << sentence_to_translate_url;
#endif // USE_GOOGLE_API_V2

#ifdef USE_MICROSOFT_TRANSLATOR_V2
  // languages available at:
  // http://api.microsofttranslator.com/V2/Http.svc/GetLanguagesForTranslate?appId=6844AE3580856D2EC7A64C79F55F11AA47CB961B
  // example:
  // http://api.microsofttranslator.com/v2/Http.svc/Translate?appId=6844AE3580856D2EC7A64C79F55F11AA47CB961B&text=Hello,%20my%20friend!&from=en&to=fr
  url << "http://api.microsofttranslator.com/v2/Http.svc/Translate?";
  url << "appId=" << application_id;
  url << "&text=" << sentence_to_translate_url;
  url << "&from=" << prefix_orig;
  url << "&to=" << prefix_dest;
#endif // USE_MICROSOFT_TRANSLATOR_V2
  // send the request
  std::string server_answer;
  string_utils::retrieve_url(url.str(), server_answer);
  //maggiePrint("server_answer:%s", server_answer.c_str());

  // extract from the tags
  int search_pos = 0;
#if defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)
  std::string ans = string_utils::extract_from_tags
      (server_answer, "\"translatedText\":\"", "\"", search_pos);
#endif // defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)

#ifdef USE_MICROSOFT_TRANSLATOR_V2
  std::string ans = string_utils::extract_from_tags
      (server_answer, ">", "<", search_pos);
#endif // USE_MICROSOFT_TRANSLATOR_V2

  // replace special characters
  string_utils::find_and_replace(ans, "&#39;", "'");
  string_utils::find_and_replace(ans, "&quot;", "'");

  return ans;
}

////////////////////////////////////////////////////////////////////////////////

void Translator::build_languages_map(LanguageMap & map) {
  // maggieDebug2("build_language_map()");
  map.insert( LanguagePair(LANGUAGE_UNKNOWN, "??") );
  map.insert( LanguagePair(LANGUAGE_AFRIKAANS, "af") );
  map.insert( LanguagePair(LANGUAGE_ALBANIAN, "sq") );
  map.insert( LanguagePair(LANGUAGE_ARABIC, "ar") );
  map.insert( LanguagePair(LANGUAGE_BULGARIAN, "bg") );
  map.insert( LanguagePair(LANGUAGE_CATALAN, "ca") );
  map.insert( LanguagePair(LANGUAGE_CHINESE_SIMPLIFIED, "zh-CN") );
  map.insert( LanguagePair(LANGUAGE_CROATIAN, "hr") );
  map.insert( LanguagePair(LANGUAGE_CZECH, "cs") );
  map.insert( LanguagePair(LANGUAGE_DANISH, "da") );
  map.insert( LanguagePair(LANGUAGE_DUTCH, "nl") );
  map.insert( LanguagePair(LANGUAGE_ENGLISH, "en") );
  map.insert( LanguagePair(LANGUAGE_ESTONIAN, "et") );
  map.insert( LanguagePair(LANGUAGE_FINNISH, "fi") );
  map.insert( LanguagePair(LANGUAGE_FRENCH, "fr") );
  map.insert( LanguagePair(LANGUAGE_GERMAN, "de") );
  map.insert( LanguagePair(LANGUAGE_GREEK, "el") );
  map.insert( LanguagePair(LANGUAGE_HAITIAN_CREOLE, "ht") );
  map.insert( LanguagePair(LANGUAGE_HINDI, "hi") );
  map.insert( LanguagePair(LANGUAGE_HUNGARIAN, "hu") );
  map.insert( LanguagePair(LANGUAGE_ICELANDIC, "is") );
  map.insert( LanguagePair(LANGUAGE_INDONESIAN, "ms") );
  map.insert( LanguagePair(LANGUAGE_ITALIAN, "it") );
  map.insert( LanguagePair(LANGUAGE_JAPANESE, "ja") );
  map.insert( LanguagePair(LANGUAGE_KOREAN, "kr") );
  map.insert( LanguagePair(LANGUAGE_LATVIAN, "lv") );
  map.insert( LanguagePair(LANGUAGE_LITUANIAN, "lt") );
  map.insert( LanguagePair(LANGUAGE_MALAY, "id") );
  map.insert( LanguagePair(LANGUAGE_MACEDONIAN, "mk") );
  map.insert( LanguagePair(LANGUAGE_NORWEGIAN, "no") );
  map.insert( LanguagePair(LANGUAGE_PERSIAN, "fa") );
  map.insert( LanguagePair(LANGUAGE_POLISH, "pl") );
  map.insert( LanguagePair(LANGUAGE_PORTUGUESE, "pt") );
  map.insert( LanguagePair(LANGUAGE_ROMANIAN, "ro") );
  map.insert( LanguagePair(LANGUAGE_RUSSIAN, "ru") );
  map.insert( LanguagePair(LANGUAGE_SERBIAN, "sr") );
  map.insert( LanguagePair(LANGUAGE_SLOVAK, "sk") );
  map.insert( LanguagePair(LANGUAGE_SLOVENIAN, "sl") );
  map.insert( LanguagePair(LANGUAGE_SPANISH, "es") );
  map.insert( LanguagePair(LANGUAGE_SWAHILI, "sw") );
  map.insert( LanguagePair(LANGUAGE_SWEDISH, "sv") );
  map.insert( LanguagePair(LANGUAGE_THAI, "th") );
  map.insert( LanguagePair(LANGUAGE_TURKISH, "tr") );
  map.insert( LanguagePair(LANGUAGE_URDU, "ur") );
  map.insert( LanguagePair(LANGUAGE_VIETNAMESE, "vi") );
  map.insert( LanguagePair(LANGUAGE_WELSH, "cy") );

}

////////////////////////////////////////////////////////////////////////////////

bool Translator::get_language_id_from_country_domain
(const LanguageMap & languages_map,
 const CountryDomain & country_domain,
 LanguageId &ans)
{
  maggieDebug3("get_language_id_from_country_domain(country_domain:'%s')", country_domain.c_str());
  ans = LANGUAGE_UNKNOWN;
  if (!map_utils::reverse_search(languages_map, country_domain, ans)) {
    printf("Cannot resolve country_domain '%s' to an int, "
           "is it a proper domain? (supported:%s) "
           "did u build the map using build_languages_map()?\n",
           country_domain.c_str(),
           string_utils::map_values_to_string(languages_map).c_str());
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool Translator::get_country_domain_from_language_id(const LanguageMap & languages_map,
                                                     const LanguageId & language_id,
                                                     CountryDomain &ans)
{
  ans = "??";
  if (!map_utils::direct_search(languages_map, language_id, ans)) {
    printf("Cannot resolve language %i to a string, "
           "is it a proper ID? (supported:%s) "
           "did u build the map using build_languages_map()?\n",
           language_id,
           string_utils::map_keys_to_string(languages_map).c_str());
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////

Translator::LanguageId Translator::detect_language(const std::string &
                                                   sentence_to_translate) {
  maggieDebug2("detect_language('%s')", sentence_to_translate.c_str());

  // prepair the string for http encoding
  std::string sentence_to_translate_url = sentence_to_translate;
  string_utils::find_and_replace(sentence_to_translate_url,
                                " ", "%20");
  string_utils::find_and_replace(sentence_to_translate_url,
                                "\n", "%0A");

  // build the URL
#if defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)
  std::ostringstream url;
  url << "https://www.googleapis.com/language/translate/v2/detect?";
  url << "key=" << google_api_key;
  url << "&prettyprint=false";
  url << "&q=" << sentence_to_translate_url;
#endif // defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)

#ifdef USE_MICROSOFT_TRANSLATOR_V2
  // http://api.microsofttranslator.com/v2/Http.svc/Detect?appId=6844AE3580856D2EC7A64C79F55F11AA47CB961B&text=Hello,%20my%20friend!
  std::ostringstream url;
  url << "http://api.microsofttranslator.com/v2/Http.svc/Detect?";
  url << "appId=" << application_id;
  url << "&text=" << sentence_to_translate_url;
#endif // USE_MICROSOFT_TRANSLATOR_V2


  // send the request
  std::string server_answer;
  string_utils::retrieve_url(url.str(), server_answer);

  // extract from the tags
  int search_pos = 0;
#if defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)
  CountryDomain country_domain = string_utils::extract_from_tags(server_answer,
                                                                "\"language\":\"",
                                                                "\"",
                                                                search_pos);
#endif // defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)

#ifdef USE_MICROSOFT_TRANSLATOR_V2
  CountryDomain country_domain = string_utils::extract_from_tags
      (server_answer, ">", "<", search_pos);
#endif // USE_MICROSOFT_TRANSLATOR_V2

  // convert the string to a language id
  LanguageMap language_map;
  build_languages_map(language_map);
  LanguageId lang_id;
  if (!get_language_id_from_country_domain(language_map, country_domain , lang_id))
    return LANGUAGE_UNKNOWN;
  return lang_id;

}

////////////////////////////////////////////////////////////////////////////////

std::string Translator::get_language_full_name(const LanguageId & language_id) {
  maggieDebug3("get_language_full_name(%i)", language_id);
  switch (language_id) {
    case Translator::LANGUAGE_AFRIKAANS: return "afrikaans";
    case Translator::LANGUAGE_ALBANIAN: return "albanian";
    case Translator::LANGUAGE_ARABIC: return "arabic";
    case Translator::LANGUAGE_BULGARIAN: return "bulgarian";
    case Translator::LANGUAGE_CATALAN: return "catalan";
    case Translator::LANGUAGE_CHINESE_SIMPLIFIED: return "simplified chinese";
    case Translator::LANGUAGE_CROATIAN: return "croatian";
    case Translator::LANGUAGE_CZECH: return "czech";
    case Translator::LANGUAGE_DANISH: return "danish";
    case Translator::LANGUAGE_DUTCH: return "dutch";
    case Translator::LANGUAGE_ENGLISH: return "english";
    case Translator::LANGUAGE_ESTONIAN: return "estonian";
    case Translator::LANGUAGE_FINNISH: return "finnish";
    case Translator::LANGUAGE_FRENCH: return "french";
    case Translator::LANGUAGE_GERMAN: return "german";
    case Translator::LANGUAGE_GREEK: return "greek";
    case Translator::LANGUAGE_HAITIAN_CREOLE: return "haitian_creole";
    case Translator::LANGUAGE_HINDI: return "hindi";
    case Translator::LANGUAGE_HUNGARIAN: return "hungarian";
    case Translator::LANGUAGE_ICELANDIC: return "icelandic";
    case Translator::LANGUAGE_INDONESIAN: return "indonesian";
    case Translator::LANGUAGE_ITALIAN: return "italian";
    case Translator::LANGUAGE_JAPANESE: return "japanese";
    case Translator::LANGUAGE_KOREAN: return "korean";
    case Translator::LANGUAGE_LATVIAN: return "latvian";
    case Translator::LANGUAGE_LITUANIAN: return "lituanian";
    case Translator::LANGUAGE_MALAY: return "malay";
    case Translator::LANGUAGE_MACEDONIAN: return "macedonian";
    case Translator::LANGUAGE_NORWEGIAN: return "norwegian";
    case Translator::LANGUAGE_PERSIAN: return "persian";
    case Translator::LANGUAGE_POLISH: return "polish";
    case Translator::LANGUAGE_PORTUGUESE: return "portuguese";
    case Translator::LANGUAGE_ROMANIAN: return "romanian";
    case Translator::LANGUAGE_RUSSIAN: return "russian";
    case Translator::LANGUAGE_SERBIAN: return "serbian";
    case Translator::LANGUAGE_SLOVAK: return "slovak";
    case Translator::LANGUAGE_SLOVENIAN: return "slovenian";
    case Translator::LANGUAGE_SPANISH: return "spanish";
    case Translator::LANGUAGE_SWAHILI: return "swahili";
    case Translator::LANGUAGE_SWEDISH: return "swedish";
    case Translator::LANGUAGE_THAI: return "thai";
    case Translator::LANGUAGE_TURKISH: return "turkish";
    case Translator::LANGUAGE_VIETNAMESE: return "vietnamese";
    case Translator::LANGUAGE_URDU: return "urdu";
    case Translator::LANGUAGE_WELSH: return "welsh";
    default:
      return "an unknwon language";
  } // end switch language_id
}

////////////////////////////////////////////////////////////////////////////////

bool Translator::_extract_country_domain(const std::string & sentence,
                                         CountryDomain & ans) {

  size_t sep_pos = sentence.find(':');
  if ( sep_pos == std::string::npos ) {
   /* maggieDebug2("The language version '%s' does not respect the syntax.",
                 sentence.c_str());*/
    return false;
  }

  ans = sentence.substr(0, sep_pos);
  return true;

}

////////////////////////////////////////////////////////////////////////////////

bool Translator::_extract_language_id(const std::string & sentence,
                                      const Translator::LanguageMap & map,
                                      Translator::LanguageId & ans) {
  maggieDebug3("_extract_language_id(sentence:'%s')", sentence.c_str());
  std::string country_domain;
  if (!_extract_country_domain(sentence, country_domain))
    return false;
  maggieDebug3("country_domain:'%s'", country_domain.c_str());
  return get_language_id_from_country_domain(map, country_domain, ans);
}

////////////////////////////////////////////////////////////////////////////////

bool Translator::_find_given_language_in_multilanguage_line(
    const std::vector<std::string> & versions,
    const Translator::LanguageId target_language,
    const Translator::LanguageMap & map,
    std::string & ans) {

  // check in each version if it was found
  CountryDomain current_version_lang_prefix;
  CountryDomain target_country_domain;
  if (!get_country_domain_from_language_id
      (map, target_language, target_country_domain))
    return false;

  std::vector<std::string> good_versions;
  for(std::vector<std::string>::const_iterator current_version = versions.begin();
      current_version != versions.end() ; ++current_version) {
    if (_extract_country_domain(*current_version, current_version_lang_prefix)
        && current_version_lang_prefix == target_country_domain) {
      // we remove the prefix and the colon
      good_versions.push_back(current_version->substr(1 + target_country_domain.size()));
    }
  } // end loop versions

  if (good_versions.empty())
    return false;

  ans = good_versions.at(rand() % good_versions.size());
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool Translator::_build_given_language_in_multilanguage_line(
    const std::string & textMultiLanguage,
    const LanguageId target_language,
    const LanguageMap & languages_map,
    std::string & ans) {
  std::vector<std::string> versions;
  string_utils::StringSplit(textMultiLanguage, "|", &versions);

  std::string sentence_good_language;
  // first search in the wanted language
  bool was_found = Translator::_find_given_language_in_multilanguage_line(
        versions, target_language, languages_map, sentence_good_language);
  if (was_found) {
    maggieDebug2("The wanted language %i was supplied by the user.",
                 target_language);
    ans = sentence_good_language;
    return true;
  }

  // if not present, search english and translate it
  was_found = Translator::_find_given_language_in_multilanguage_line(
        versions, Translator::LANGUAGE_ENGLISH, languages_map, sentence_good_language);
  if (was_found) {
    maggieDebug2("LANGUAGE_ENGLISH was supplied by the user. "
                 "Translating from that to %i",
                 target_language);
    ans = Translator::translate(sentence_good_language,
                                Translator::LANGUAGE_ENGLISH,
                                target_language);
    return true;
  }

  // otherwise, translate the first that comes
  for(std::vector<std::string>::const_iterator version = versions.begin();
      version != versions.end() ; ++version) {

    // extract the language key
    Translator::LanguageId version_language_id;
    bool language_id_found;
    language_id_found = Translator::_extract_language_id(*version, languages_map, version_language_id);
    if (!language_id_found)
      continue;

    // extract the corresponding sentence
    was_found = Translator::_find_given_language_in_multilanguage_line(
          versions, version_language_id, languages_map, sentence_good_language);
    if (was_found) {
      maggieDebug2("%i was supplied by the user. "
                   "Translating from that ('%s') to %i",
                   version_language_id,
                   sentence_good_language.c_str(),
                   target_language);
      ans = Translator::translate(sentence_good_language,
                                  version_language_id,
                                  target_language);
      return true;
    }
  } // end loop version

  // if everything failed, assume it is current language
  /*maggiePrint("I couldn't detect any known language in the sentence '%s'. "
              "Did you respect the sayTextNL() syntax ?",
              textMultiLanguage.c_str());*/

  ans = textMultiLanguage;
  // ans = versions.front();
  return false;
}

////////////////////////////////////////////////////////////////////////////////

/** Traduce una frase a Gibberish Language
    The general process you'll want to follow is:
    - split the string
    - loop through each word
    - for each word, move the first letter to the end, and all other letters up one position in the string.
    - put all the words back together as a single string.
    **/
std::string Translator::_translateToGibberishLanguage(const std::string & stringOriginal){

  std::string manip = "",newstring = "";
  std::vector<std::string> words;

  string_utils::StringSplit(stringOriginal, " ", &words);

  for(unsigned int i=0;i<words.size();i++)
  {
    if(i!=(words.size()-1))
    {
      manip = words[i].substr(1) + words[i].substr(0,1);
      words[i] = manip;
    }
    else
    {
      manip = words[i].substr(1,(words[i].length()-2)) + words[i].substr(0,1)+ words[i].substr((words[i].length()-1),1);
      words[i] = manip;
    }
  }
  for(unsigned int i=0;i<words.size();i++)
  {
    newstring += words[i];
    if(i!=(words.size()-1))
      newstring += " ";
  }

  return newstring;
}


