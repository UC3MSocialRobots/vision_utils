#ifndef TRANSLATOR_H
#define TRANSLATOR_H

// std
#include <map>
#include <string>
#include <vector>

//#include "network/NetworkUtils.h"
#include "vision_utils/find_and_replace.h"
#include "vision_utils/string_split.h"

namespace vision_utils {

//cut:build_languages_map
typedef std::string CountryDomain;
typedef int LanguageId;
typedef std::map<LanguageId, CountryDomain> LanguageMap;
typedef LanguageMap::value_type LanguagePair;

enum { // sorted list please!
  LANGUAGE_UNKNOWN = -1,
  LANGUAGE_LAST_USED = 0,
  //
  LANGUAGE_AFRIKAANS,
  LANGUAGE_ALBANIAN,
  LANGUAGE_ARABIC,
  LANGUAGE_BULGARIAN,
  LANGUAGE_CATALAN,
  LANGUAGE_CHINESE_SIMPLIFIED,
  LANGUAGE_CROATIAN,
  LANGUAGE_CZECH,
  LANGUAGE_DANISH,
  LANGUAGE_DUTCH,
  LANGUAGE_ENGLISH,
  LANGUAGE_ESTONIAN,
  LANGUAGE_FINNISH,
  LANGUAGE_FRENCH,
  LANGUAGE_GERMAN,
  LANGUAGE_GREEK,
  LANGUAGE_HAITIAN_CREOLE,
  LANGUAGE_HINDI,
  LANGUAGE_HUNGARIAN,
  LANGUAGE_ICELANDIC,
  LANGUAGE_INDONESIAN,
  LANGUAGE_ITALIAN,
  LANGUAGE_JAPANESE,
  LANGUAGE_KOREAN,
  LANGUAGE_LATVIAN,
  LANGUAGE_LITUANIAN,
  LANGUAGE_MALAY,
  LANGUAGE_MACEDONIAN,
  LANGUAGE_NORWEGIAN,
  LANGUAGE_PERSIAN,
  LANGUAGE_POLISH,
  LANGUAGE_PORTUGUESE,
  LANGUAGE_ROMANIAN,
  LANGUAGE_RUSSIAN,
  LANGUAGE_SERBIAN,
  LANGUAGE_SLOVAK,
  LANGUAGE_SLOVENIAN,
  LANGUAGE_SPANISH,
  LANGUAGE_SWAHILI,
  LANGUAGE_SWEDISH,
  LANGUAGE_THAI,
  LANGUAGE_TURKISH,
  LANGUAGE_VIETNAMESE,
  LANGUAGE_URDU,
  LANGUAGE_WELSH
};

////////////////////////////////////////////////////////////////////////////////
/*! a function to populate a map with the Google languages */
void build_languages_map(LanguageMap & map) {
  // printf("build_language_map()\n");
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
//cut:get_language_id_from_country_domain
/*! make an inversed search in the language map
  \return true if success */
bool get_language_id_from_country_domain(const LanguageMap & languages_map,
                                         const CountryDomain & country_domain,
                                         LanguageId & ans)
{
  //printf("get_language_id_from_country_domain(country_domain:'%s')\n", country_domain.c_str());
  ans = LANGUAGE_UNKNOWN;
  if (!vision_utils::reverse_search(languages_map, country_domain, ans)) {
    printf("Cannot resolve country_domain '%s' to an int, "
           "is it a proper domain? (supported:%s) "
           "did u build the map using build_languages_map()?\n",
           country_domain.c_str(),
           vision_utils::map_values_to_string(languages_map).c_str());
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
//cut:get_country_domain_from_language_id
/*! make a direct search in the language map
  \return true if success */
bool get_country_domain_from_language_id(const LanguageMap & languages_map,
                                         const LanguageId & language_id,
                                         CountryDomain & ans)
{
  ans = "??";
  if (!vision_utils::direct_search(languages_map, language_id, ans)) {
    printf("Cannot resolve language %i to a string, "
           "is it a proper ID? (supported:%s) "
           "did u build the map using build_languages_map()?\n",
           language_id,
           vision_utils::map_keys_to_string(languages_map).c_str());
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
//cut:get_language_full_name
/*! \return the full name of a language.
  \example "english" for LANGUAGE_ENGLISH */
std::string get_language_full_name(const LanguageId & language_id) {
  //printf("get_language_full_name(%i)\n", language_id);
  switch (language_id) {
  case vision_utils::LANGUAGE_AFRIKAANS: return "afrikaans";
  case vision_utils::LANGUAGE_ALBANIAN: return "albanian";
  case vision_utils::LANGUAGE_ARABIC: return "arabic";
  case vision_utils::LANGUAGE_BULGARIAN: return "bulgarian";
  case vision_utils::LANGUAGE_CATALAN: return "catalan";
  case vision_utils::LANGUAGE_CHINESE_SIMPLIFIED: return "simplified chinese";
  case vision_utils::LANGUAGE_CROATIAN: return "croatian";
  case vision_utils::LANGUAGE_CZECH: return "czech";
  case vision_utils::LANGUAGE_DANISH: return "danish";
  case vision_utils::LANGUAGE_DUTCH: return "dutch";
  case vision_utils::LANGUAGE_ENGLISH: return "english";
  case vision_utils::LANGUAGE_ESTONIAN: return "estonian";
  case vision_utils::LANGUAGE_FINNISH: return "finnish";
  case vision_utils::LANGUAGE_FRENCH: return "french";
  case vision_utils::LANGUAGE_GERMAN: return "german";
  case vision_utils::LANGUAGE_GREEK: return "greek";
  case vision_utils::LANGUAGE_HAITIAN_CREOLE: return "haitian_creole";
  case vision_utils::LANGUAGE_HINDI: return "hindi";
  case vision_utils::LANGUAGE_HUNGARIAN: return "hungarian";
  case vision_utils::LANGUAGE_ICELANDIC: return "icelandic";
  case vision_utils::LANGUAGE_INDONESIAN: return "indonesian";
  case vision_utils::LANGUAGE_ITALIAN: return "italian";
  case vision_utils::LANGUAGE_JAPANESE: return "japanese";
  case vision_utils::LANGUAGE_KOREAN: return "korean";
  case vision_utils::LANGUAGE_LATVIAN: return "latvian";
  case vision_utils::LANGUAGE_LITUANIAN: return "lituanian";
  case vision_utils::LANGUAGE_MALAY: return "malay";
  case vision_utils::LANGUAGE_MACEDONIAN: return "macedonian";
  case vision_utils::LANGUAGE_NORWEGIAN: return "norwegian";
  case vision_utils::LANGUAGE_PERSIAN: return "persian";
  case vision_utils::LANGUAGE_POLISH: return "polish";
  case vision_utils::LANGUAGE_PORTUGUESE: return "portuguese";
  case vision_utils::LANGUAGE_ROMANIAN: return "romanian";
  case vision_utils::LANGUAGE_RUSSIAN: return "russian";
  case vision_utils::LANGUAGE_SERBIAN: return "serbian";
  case vision_utils::LANGUAGE_SLOVAK: return "slovak";
  case vision_utils::LANGUAGE_SLOVENIAN: return "slovenian";
  case vision_utils::LANGUAGE_SPANISH: return "spanish";
  case vision_utils::LANGUAGE_SWAHILI: return "swahili";
  case vision_utils::LANGUAGE_SWEDISH: return "swedish";
  case vision_utils::LANGUAGE_THAI: return "thai";
  case vision_utils::LANGUAGE_TURKISH: return "turkish";
  case vision_utils::LANGUAGE_VIETNAMESE: return "vietnamese";
  case vision_utils::LANGUAGE_URDU: return "urdu";
  case vision_utils::LANGUAGE_WELSH: return "welsh";
  default:
    return "an unknwon language";
  } // end switch language_id
}

////////////////////////////////////////////////////////////////////////////////
//cut:detect_language
//! here is defined the active translator engine
//#define USE_GOOGLE_API_V1 // deprecated since june 2011
//#define USE_GOOGLE_API_V2 // costly since december 2011
#define USE_MICROSOFT_TRANSLATOR_V2 // aaaargh, microsoft!

#ifdef USE_GOOGLE_API_V2
#define GOOGLE_API_KEY "AIzaSyDugyKvZraqZjIh9038_HZZewrVQu1NwQ0";
#endif // USE_GOOGLE_API_V2

#ifdef USE_MICROSOFT_TRANSLATOR_V2
#define MICROSOFT_APPLICATION_ID "6844AE3580856D2EC7A64C79F55F11AA47CB961B";
#endif // USE_MICROSOFT_TRANSLATOR_V2


/*! calls the google appi to detect the language */
LanguageId detect_language(const std::string & sentence_to_translate) {
  printf("detect_language('%s')\n", sentence_to_translate.c_str());

  // prepair the string for http encoding
  std::string sentence_to_translate_url = sentence_to_translate;
  vision_utils::find_and_replace(sentence_to_translate_url,
                                 " ", "%20");
  vision_utils::find_and_replace(sentence_to_translate_url,
                                 "\n", "%0A");

  // build the URL
#if defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)
  std::ostringstream url;
  url << "https://www.googleapis.com/language/translate/v2/detect?";
  url << "key=" << GOOGLE_API_KEY;
  url << "&prettyprint=false";
  url << "&q=" << sentence_to_translate_url;
#endif // defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)

#ifdef USE_MICROSOFT_TRANSLATOR_V2
  // http://api.microsofttranslator.com/v2/Http.svc/Detect?appId=6844AE3580856D2EC7A64C79F55F11AA47CB961B&text=Hello,%20my%20friend!
  std::ostringstream url;
  url << "http://api.microsofttranslator.com/v2/Http.svc/Detect?";
  url << "appId=" << MICROSOFT_APPLICATION_ID;
  url << "&text=" << sentence_to_translate_url;
#endif // USE_MICROSOFT_TRANSLATOR_V2


  // send the request
  std::string server_answer;
  vision_utils::retrieve_url(url.str(), server_answer);

  // extract from the tags
  int search_pos = 0;
#if defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)
  CountryDomain country_domain = vision_utils::extract_from_tags(server_answer,
                                                                 "\"language\":\"",
                                                                 "\"",
                                                                 search_pos);
#endif // defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)

#ifdef USE_MICROSOFT_TRANSLATOR_V2
  CountryDomain country_domain = vision_utils::extract_from_tags
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
//cut:translate
//! \return sentence_to_translate if fails
std::string translate(const std::string & sentence_to_translate,
                      const LanguageId & language_orig,
                      const LanguageId & language_dest) {

  printf("translate(sentence:'%s'' : %s -> %s)\n",
           sentence_to_translate.c_str(),
           get_language_full_name(language_orig).c_str(),
           get_language_full_name(language_dest).c_str());

  // don't translate if the languages are identical
  if (language_orig == language_dest)
    return sentence_to_translate;

  // prepair the string for http encoding
  std::string sentence_to_translate_url = sentence_to_translate;
  vision_utils::find_and_replace(sentence_to_translate_url,
                                 " ", "%20");
  vision_utils::find_and_replace(sentence_to_translate_url,
                                 "\n", "%0A");

  // find the ides
  LanguageMap language_map;
  build_languages_map(language_map);
  // autodetect language if needed
  LanguageId language_orig_detect = language_orig;
  if (language_orig == LANGUAGE_UNKNOWN)
    language_orig_detect = detect_language(sentence_to_translate);
  if (language_orig_detect == LANGUAGE_UNKNOWN) { // detection failed -> quit
    printf("Impossible to detect the language of '%s'\n",
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
  url << "key=" << GOOGLE_API_KEY;
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
  url << "appId=" << MICROSOFT_APPLICATION_ID;
  url << "&text=" << sentence_to_translate_url;
  url << "&from=" << prefix_orig;
  url << "&to=" << prefix_dest;
#endif // USE_MICROSOFT_TRANSLATOR_V2
  // send the request
  std::string server_answer;
  vision_utils::retrieve_url(url.str(), server_answer);
  //printf("server_answer:%s\n", server_answer.c_str());

  // extract from the tags
  int search_pos = 0;
#if defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)
  std::string ans = vision_utils::extract_from_tags
      (server_answer, "\"translatedText\":\"", "\"", search_pos);
#endif // defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)

#ifdef USE_MICROSOFT_TRANSLATOR_V2
  std::string ans = vision_utils::extract_from_tags
      (server_answer, ">", "<", search_pos);
#endif // USE_MICROSOFT_TRANSLATOR_V2

  // replace special characters
  vision_utils::find_and_replace(ans, "&#39;", "'");
  vision_utils::find_and_replace(ans, "&quot;", "'");

  return ans;
}

////////////////////////////////////////////////////////////////////////////////
//// functions for string processing
//cut:extract_country_domain
/*!
  extract the language indicator at the beginning of the sentence
  \param sentence a sentence starting with an indicator
  \param ans will be populated
  \return true if success
  \example extract_language_id("en:foo", ans)
  will return true and ans = "en"
  */
bool extract_country_domain(const std::string & sentence,
                             CountryDomain & ans) {

  size_t sep_pos = sentence.find(':');
  if ( sep_pos == std::string::npos ) {
    /* printf("The language version '%s' does not respect the syntax.\n",
                 sentence.c_str());*/
    return false;
  }

  ans = sentence.substr(0, sep_pos);
  return true;

}

//cut:extract_language_id

/*!
  extract the language indicator and makes an inverse lookup in the map
  \param ans will be populated
  \return true if success
  \example sentence="en:foo" return LANGUAGE_ENGLISH
  */
bool extract_language_id(const std::string & sentence,
                          const LanguageMap & map,
                          LanguageId & ans) {
  //printf("extract_language_id(sentence:'%s')\n", sentence.c_str());
  std::string country_domain;
  if (!extract_country_domain(sentence, country_domain))
    return false;
  //printf("country_domain:'%s'\n", country_domain.c_str());
  return get_language_id_from_country_domain(map, country_domain, ans);
}

////////////////////////////////////////////////////////////////////////////////
//cut:find_given_language_in_multilanguage_line
/*!
  \param versions a vector of strings,
  each string is the country domain and the sentence in this language.
  Multiple instances of a given language are supported,
  one of these will be chosen randomly.
  \example versions={"en:Hello","en:Hi","fr:Salut"}
           if (target_language == LANGUAGE_ENGLISH),
             will return randomly "Hello" or "Hi"
           if (target_language == LANGUAGE_FRENCH)
             will return "Salut"
  \return true if success

  \example if target_language == LANGUAGE_ENGLISH,
  will search a sentence that starts with "en:"
  */
bool find_given_language_in_multilanguage_line(
    const std::vector<std::string> & versions,
    const LanguageId target_language,
    const LanguageMap & map,
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
    if (extract_country_domain(*current_version, current_version_lang_prefix)
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
//cut:build_given_language_in_multilanguage_line
/*! first, try to see if the given language is given
   *  in \arg textMultiLanguage.
   * If it is not, translate from one of the given languages,
   * by default trying with the english version (en:),
   * if english not available with whatever is given.
   * \example "es:hola|en:hi|fr:bonjour"
   *
   * Multiple instances of a given language are supported,
   * one of these will be chosen randomly.
   * \example textMultiLanguage="en:Hello|en:Hi|fr:Salut"
   *            if (target_language == LANGUAGE_ENGLISH),
   *              will return randomly "Hello" or "Hi"
   *            if (target_language == LANGUAGE_FRENCH)
   *              will return "Salut"
   */
bool build_given_language_in_multilanguage_line(
    const std::string & textMultiLanguage,
    const LanguageId target_language,
    const LanguageMap & languages_map,
    std::string & ans) {
  std::vector<std::string> versions;
  vision_utils::StringSplit(textMultiLanguage, "|", &versions);

  std::string sentence_good_language;
  // first search in the wanted language
  bool was_found = vision_utils::find_given_language_in_multilanguage_line(
        versions, target_language, languages_map, sentence_good_language);
  if (was_found) {
    printf("The wanted language %i was supplied by the user.\n",
             target_language);
    ans = sentence_good_language;
    return true;
  }

  // if not present, search english and translate it
  was_found = vision_utils::find_given_language_in_multilanguage_line(
        versions, vision_utils::LANGUAGE_ENGLISH, languages_map, sentence_good_language);
  if (was_found) {
    printf("LANGUAGE_ENGLISH was supplied by the user. "
             "Translating from that to %i\n",
             target_language);
    ans = vision_utils::translate(sentence_good_language,
                                  vision_utils::LANGUAGE_ENGLISH,
                                  target_language);
    return true;
  }

  // otherwise, translate the first that comes
  for(std::vector<std::string>::const_iterator version = versions.begin();
      version != versions.end() ; ++version) {

    // extract the language key
    vision_utils::LanguageId version_language_id;
    bool language_id_found;
    language_id_found = vision_utils::extract_language_id
        (*version, languages_map, version_language_id);
    if (!language_id_found)
      continue;

    // extract the corresponding sentence
    was_found = vision_utils::find_given_language_in_multilanguage_line(
          versions, version_language_id, languages_map, sentence_good_language);
    if (was_found) {
      printf("%i was supplied by the user. "
               "Translating from that ('%s') to %i\n",
               version_language_id,
               sentence_good_language.c_str(),
               target_language);
      ans = vision_utils::translate(sentence_good_language,
                                    version_language_id,
                                    target_language);
      return true;
    }
  } // end loop version

  // if everything failed, assume it is current language
  /*printf("I couldn't detect any known language in the sentence '%s'. "
              "Did you respect the sayTextNL() syntax ?\n",
              textMultiLanguage.c_str());*/

  ans = textMultiLanguage;
  // ans = versions.front();
  return false;
}

////////////////////////////////////////////////////////////////////////////////
//cut:translateToGibberishLanguage
/** Traduce una frase a Gibberish Language
      The general process you'll want to follow is:
      - split the string
      - loop through each word
      - for each word, move the first letter to the end, and all other letters up one position in the string.
      - put all the words back together as a single string.
      **/
std::string translateToGibberishLanguage(const std::string & stringSource){

  std::string manip = "",newstring = "";
  std::vector<std::string> words;

  vision_utils::StringSplit(stringSource, " ", &words);

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

} // end namespace vision_utils

#endif // TRANSLATOR_H
