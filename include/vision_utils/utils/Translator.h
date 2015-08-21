#ifndef TRANSLATOR_H
#define TRANSLATOR_H

// std
#include <map>
#include <string>
#include <vector>

class Translator {
public:
  typedef std::string CountryDomain;
  typedef int LanguageId;
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

  typedef std::map<LanguageId, CountryDomain> LanguageMap;
  typedef LanguageMap::value_type LanguagePair;

  /*! a static function to populate a map with the Google languages */
  static void build_languages_map(LanguageMap & map);

  /*! make an inversed search in the language map
  \return true if success */
  static bool get_language_id_from_country_domain(const LanguageMap & languages_map,
                                                  const CountryDomain & country_domain,
                                                  LanguageId & ans);

  /*! make a direct search in the language map
  \return true if success */
  static bool get_country_domain_from_language_id(const LanguageMap & languages_map,
                                                  const LanguageId & language_id,
                                                  CountryDomain & ans);

  /*! \return the full name of a language.
  \example "english" for LANGUAGE_ENGLISH */
  static std::string get_language_full_name(const LanguageId & language_id);

  ////////////////////////////////////////////////////////////////////////////

  //! \return sentence_to_translate if fails
  static std::string translate(const std::string & sentence_to_translate,
                               const LanguageId & language_orig,
                               const LanguageId & language_dest);

  /*! calls the google appi to detect the language */
  static LanguageId detect_language(const std::string & sentence_to_translate);

  //// functions for string processing


  /*!
  extract the language indicator at the beginning of the sentence
  \param sentence a sentence starting with an indicator
  \param ans will be populated
  \return true if success
  \example _extract_language_id("en:foo", ans)
  will return true and ans = "en"
  */
  static bool _extract_country_domain(const std::string & sentence,
                                      CountryDomain & ans);

  /*!
  extract the language indicator and makes an inverse lookup in the map
  \param ans will be populated
  \return true if success
  \example sentence="en:foo" return LANGUAGE_ENGLISH
  */
  static bool _extract_language_id(const std::string & sentence,
                                   const LanguageMap & map,
                                   LanguageId & ans);

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
  static bool _find_given_language_in_multilanguage_line(
      const std::vector<std::string> & versions,
      const LanguageId target_language,
      const LanguageMap & map,
      std::string & ans);

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
  static bool _build_given_language_in_multilanguage_line(
      const std::string & textMultiLanguage,
      const LanguageId target_language,
      const LanguageMap & map,
      std::string & ans);

  /** Traduce una frase a Gibberish Language mediante un servicio online*/
  static std::string _translateToGibberishLanguage(const std::string & stringSource);
};

#endif // TRANSLATOR_H
