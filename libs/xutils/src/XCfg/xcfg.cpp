/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 * USA
 *
 */
/********************************************************************
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *USA
 *
 ********************************************************************/

/*
 * $Id$
 */

#include "xcfg.h"
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <locale.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "global.h"

#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif

// extern int global_playerport;

const char *COLOR_DATABASE[5] = {"/usr/X11R6/lib/X11/rgb.txt",
                                 "/usr/share/X11/rgb.txt", "/etc/X11/rgb.txt",
                                 "/usr/lib/X11/rgb.txt", NULL};

///////////////////////////////////////////////////////////////////////////
// the isblank() macro is not standard - it's a GNU extension
// and it doesn't work for me, so here's an implementation - rtv
#ifndef isblank
#define isblank(a) (a == ' ' || a == '\t')
#endif

///////////////////////////////////////////////////////////////////////////
// Useful macros for dumping parser errors
#define TOKEN_ERR(z, l)                                                        \
  fprintf(stderr, "%s:%d error: " z "\n", this->filename.c_str(), l)
#define PARSE_ERR(z, l)                                                        \
  fprintf(stderr, "%s:%d error: " z "\n", this->filename.c_str(), l)

#define CONFIG_WARN1(z, line, a)                                               \
  fprintf(stderr, "%s:%d warning: " z "\n", this->filename.c_str(), line, a)
#define CONFIG_WARN2(z, line, a, b)                                            \
  fprintf(stderr, "%s:%d warning: " z "\n", this->filename.c_str(), line, a, b)

#define CONFIG_ERR1(z, line, a)                                                \
  fprintf(stderr, "%s:%d error: " z "\n", this->filename.c_str(), line, a)
#define CONFIG_ERR2(z, line, a, b)                                             \
  fprintf(stderr, "%s:%d error: " z "\n", this->filename.c_str(), line, a, b)

///////////////////////////////////////////////////////////////////////////
// Default constructor
ConfigFile::ConfigFile(std::string filePath) {
  filePath = std::string(CONFIG_PREFIX) + filePath;
  this->InitFields();
  this->Load(filePath.c_str());
}

/// Alternate constructor, used when not loading from a file
ConfigFile::ConfigFile() { this->InitFields(); }

void ConfigFile::InitFields() {

  this->token_size = 0;
  this->token_count = 0;
  this->tokens = NULL;

  this->macro_count = 0;
  this->macro_size = 0;
  this->macros = NULL;

  this->section_count = 0;
  this->section_size = 0;
  this->sections = NULL;

  this->field_count = 0;
  this->field_size = 0;
  this->fields = NULL;

  // Set defaults units
  this->unit_length = 1.0;
  this->unit_angle = M_PI / 180;

  if (!setlocale(LC_ALL, "POSIX"))
    fputs("Warning: failed to setlocale(); config file may not be parse "
          "correctly\n",
          stderr);
}

///////////////////////////////////////////////////////////////////////////
// Destructor
ConfigFile::~ConfigFile() {
  ClearFields();
  ClearMacros();
  ClearSections();
  ClearTokens();
}

///////////////////////////////////////////////////////////////////////////
// Load world from file
bool ConfigFile::Load(const char *_filename) {
  // Shouldnt call load more than once,
  // so this should be null.
  assert(this->filename.empty());
  this->filename = strdup(_filename);

  // Open the file
  printf("Prepare to open file: %s\r\n", this->filename.c_str());
  FILE *file = fopen(this->filename.c_str(), "r");
  if (!file) {
    printf("unable to open world file %s : %s\r\n", this->filename.c_str(),
           strerror(errno));
    // fclose(file);
    return false;
  }
  ClearTokens();

  // Read tokens from the file
  if (!LoadTokens(file, 0)) {
    // DumpTokens();
    printf("LoadTokens failed\r\n");
    fclose(file);
    return false;
  }

  // Parse the tokens to identify sections
  if (!ParseTokens()) {
    // DumpTokens();
    printf("ParseTokens failed\r\n");
    fclose(file);
    return false;
  }

  // Dump contents and exit if this file is meant for debugging only.
  if (ReadInt(0, "test", 0) != 0) {
    printf("this is a test file; quitting\r\n");
    DumpTokens();
    DumpMacros();
    DumpSections();
    DumpFields();
    fclose(file);
    return false;
  }

  // Work out what the length units are
  const char *unit = ReadString(0, "unit_length", "m");
  if (strcmp(unit, "m") == 0)
    this->unit_length = 1.0;
  else if (strcmp(unit, "cm") == 0)
    this->unit_length = 0.01;
  else if (strcmp(unit, "mm") == 0)
    this->unit_length = 0.001;

  // Work out what the angle units are
  unit = ReadString(0, "unit_angle", "degrees");
  if (strcmp(unit, "degrees") == 0)
    this->unit_angle = M_PI / 180;
  else if (strcmp(unit, "radians") == 0)
    this->unit_angle = 1;

  // DumpTokens();
  fclose(file);
  return true;
}

///////////////////////////////////////////////////////////////////////////
/// Add a (name,value) pair directly into the database, without
/// reading from a file.  The (name,value) goes into the "global" section.
void ConfigFile::InsertFieldValue(int index, const char *name,
                                  const char *value) {
  // Work out what the length units are
  if (!strcmp(name, "unit_length")) {
    if (strcmp(value, "m") == 0)
      this->unit_length = 1.0;
    else if (strcmp(value, "cm") == 0)
      this->unit_length = 0.01;
    else if (strcmp(value, "mm") == 0)
      this->unit_length = 0.001;
    return;
  }

  // Work out what the angle units are
  if (!strcmp(name, "unit_angle")) {
    if (strcmp(value, "degrees") == 0)
      this->unit_angle = M_PI / 180;
    else if (strcmp(value, "radians") == 0)
      this->unit_angle = 1;
    return;
  }

  // AddField checks whether the field already exists
  // printf("before field_count=%d\r\n", this->field_count);
  int field = this->AddField(0, name, 1);
  // printf("after field_count=%d\r\n", this->field_count);
  // printf("before token_count=%d\r\n", this->token_count);
  this->AddToken(ConfigFile::TokenWord, value, 0);
  // printf("after token_count=%d\r\n", this->token_count);
  this->AddFieldValue(field, index, this->token_count - 1);
}

///////////////////////////////////////////////////////////////////////////
// Save world to file
bool ConfigFile::Save(const char *filename) {
  // Debugging
  // DumpFields();

  // If no filename is supplied, use default
  if (!filename)
    filename = this->filename.c_str();

  // Open file
  FILE *file = fopen(filename, "w+");
  if (!file) {
    printf("unable to open world file %s : %s\r\n", filename, strerror(errno));
    return false;
  }

  // Write the current set of tokens to the file
  if (!SaveTokens(file)) {
    fclose(file);
    return false;
  }

  fclose(file);
  return true;
}

///////////////////////////////////////////////////////////////////////////
// Check for unused fields and print warnings
bool ConfigFile::WarnUnused() {
  bool unused = false;
  for (int i = 0; i < this->field_count; i++) {
    Field *field = this->fields + i;

    if (field->value_count <= 1) {
      if (!field->useds[0]) {
        unused = true;
        CONFIG_WARN1("field [%s] is defined but not used", field->line,
                     field->name);
      }
    } else {
      for (int j = 0; j < field->value_count; j++) {
        if (!field->useds[j]) {
          unused = true;
          CONFIG_WARN2("field [%s] has unused element %d", field->line,
                       field->name, j);
        }
      }
    }
  }
  return unused;
}

///////////////////////////////////////////////////////////////////////////
// Load tokens from a file.
bool ConfigFile::LoadTokens(FILE *file, int include) {
  int ch;
  int line;
  char token[256];

  line = 1;

  while (true) {
    ch = fgetc(file);
    if (ch == EOF)
      break;

    if ((char)ch == '#') {
      ungetc(ch, file);
      if (!LoadTokenComment(file, &line, include))
        return false;
    } else if (isalpha(ch)) {
      ungetc(ch, file);
      if (!LoadTokenWord(file, &line, include))
        return false;
    } else if (strchr("+-.0123456789", ch)) {
      ungetc(ch, file);
      if (!LoadTokenNum(file, &line, include))
        return false;
    } else if (isblank(ch)) {
      ungetc(ch, file);
      if (!LoadTokenSpace(file, &line, include))
        return false;
    } else if (ch == '"') {
      ungetc(ch, file);
      if (!LoadTokenString(file, &line, include))
        return false;
    } else if (strchr("(", ch)) {
      token[0] = ch;
      token[1] = 0;
      AddToken(TokenOpenSection, token, include);
    } else if (strchr(")", ch)) {
      token[0] = ch;
      token[1] = 0;
      AddToken(TokenCloseSection, token, include);
    } else if (strchr("[", ch)) {
      token[0] = ch;
      token[1] = 0;
      AddToken(TokenOpenTuple, token, include);
    } else if (strchr("]", ch)) {
      token[0] = ch;
      token[1] = 0;
      AddToken(TokenCloseTuple, token, include);
    } else if (0x0d == ch) {
      ch = fgetc(file);
      if (0x0a != ch)
        ungetc(ch, file);
      line++;
      AddToken(TokenEOL, "\n", include);
    } else if (0x0a == ch) {
      ch = fgetc(file);
      if (0x0d != ch)
        ungetc(ch, file);
      line++;
      AddToken(TokenEOL, "\n", include);
    } else {
      TOKEN_ERR("syntax error", line);
      return false;
    }
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////
// Read in a comment token
bool ConfigFile::LoadTokenComment(FILE *file, int *line, int include) {
  char token[1024];
  int len;
  int ch;

  len = 0;
  memset(token, 0, sizeof(token));

  while (true) {
    ch = fgetc(file);

    if (ch == EOF) {
      AddToken(TokenComment, token, include);
      return true;
    } else if (0x0a == ch || 0x0d == ch) {
      ungetc(ch, file);
      AddToken(TokenComment, token, include);
      return true;
    } else
      token[len++] = ch;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////
// Read in a word token
bool ConfigFile::LoadTokenWord(FILE *file, int *line, int include) {
  char token[1024];
  int len;
  int ch;

  len = 0;
  memset(token, 0, sizeof(token));

  while (true) {
    ch = fgetc(file);

    if (ch == EOF) {
      AddToken(TokenWord, token, include);
      return true;
    } else if (isalpha(ch) || isdigit(ch) || strchr(".-_[]:", ch)) {
      token[len++] = ch;
    } else {
      if (strcmp(token, "include") == 0) {
        ungetc(ch, file);
        AddToken(TokenWord, token, include);
        if (!LoadTokenInclude(file, line, include))
          return false;
      } else if (strcmp(token, "true") == 0 || strcmp(token, "false") == 0 ||
                 strcmp(token, "yes") == 0 || strcmp(token, "no") == 0) {
        ungetc(ch, file);
        AddToken(TokenBool, token, include);
      } else {
        ungetc(ch, file);
        AddToken(TokenWord, token, include);
      }
      return true;
    }
  }
  assert(false);
  return false;
}

///////////////////////////////////////////////////////////////////////////
// Load an include token; this will load the include file.
bool ConfigFile::LoadTokenInclude(FILE *file, int *line, int include) {
  int ch;
  const char *filename;
  char *fullpath;

  ch = fgetc(file);

  if (ch == EOF) {
    TOKEN_ERR("incomplete include statement", *line);
    return false;
  } else if (!isblank(ch)) {
    TOKEN_ERR("syntax error in include statement", *line);
    return false;
  }

  ungetc(ch, file);
  if (!LoadTokenSpace(file, line, include))
    return false;

  ch = fgetc(file);

  if (ch == EOF) {
    TOKEN_ERR("incomplete include statement", *line);
    return false;
  } else if (ch != '"') {
    TOKEN_ERR("syntax error in include statement", *line);
    return false;
  }

  ungetc(ch, file);
  if (!LoadTokenString(file, line, include))
    return false;

  // This is the basic filename
  filename = GetTokenValue(this->token_count - 1);

  // Now do some manipulation.  If its a relative path,
  // we append the path of the world file.
  if (filename[0] == '/' || filename[0] == '~') {
    fullpath = strdup(filename);
  } else if (this->filename[0] == '/' || this->filename[0] == '~') {
    // Note that dirname() modifies the contents, so
    // we need to make a copy of the filename.
    // There's no bounds-checking, but what the heck.
    char *tmp = strdup(this->filename.c_str());
    fullpath = (char *)malloc(PATH_MAX);
    memset(fullpath, 0, PATH_MAX);
    strcat(fullpath, dirname(tmp));
    strcat(fullpath, "/");
    strcat(fullpath, filename);
    assert(strlen(fullpath) + 1 < PATH_MAX);
    free(tmp);
  } else {
    // Note that dirname() modifies the contents, so
    // we need to make a copy of the filename.
    // There's no bounds-checking, but what the heck.
    char *tmp = strdup(this->filename.c_str());
    fullpath = (char *)malloc(PATH_MAX);
    char *ret = getcwd(fullpath, PATH_MAX);
    if (ret == NULL) {
      printf("Failed to get working directory\r\n");
      assert(false);
      // TODO: handle this error properly
    }
    strcat(fullpath, "/");
    strcat(fullpath, dirname(tmp));
    strcat(fullpath, "/");
    strcat(fullpath, filename);
    assert(strlen(fullpath) + 1 < PATH_MAX);
    free(tmp);
  }

  // Open the include file
  FILE *infile = fopen(fullpath, "r");
  if (!infile) {
    printf("unable to open include file %s : %s\r\n", fullpath,
           strerror(errno));
    free(fullpath);
    return false;
  }

  // Add an EOL to stop parsing of the include statement
  AddToken(TokenEOL, "\n", include);

  // Read tokens from the file
  if (!LoadTokens(infile, include + 1)) {
    // DumpTokens();
    free(fullpath);
    return false;
  }

  free(fullpath);
  return true;
}

///////////////////////////////////////////////////////////////////////////
// Read in a number token
bool ConfigFile::LoadTokenNum(FILE *file, int *line, int include) {
  char token[1024];
  int len;
  int ch;

  len = 0;
  memset(token, 0, sizeof(token));

  while (true) {
    ch = fgetc(file);

    if (ch == EOF) {
      AddToken(TokenNum, token, include);
      return true;
    } else if (strchr("+-.0123456789", ch)) {
      token[len++] = ch;
    } else {
      AddToken(TokenNum, token, include);
      ungetc(ch, file);
      return true;
    }
  }
  assert(false);
  return false;
}

///////////////////////////////////////////////////////////////////////////
// Read in a string token
bool ConfigFile::LoadTokenString(FILE *file, int *line, int include) {
  int ch;
  int len;
  char token[1024];

  len = 0;
  memset(token, 0, sizeof(token));

  ch = fgetc(file);

  while (true) {
    ch = fgetc(file);

    if (ch == EOF || ch == '\n') {
      TOKEN_ERR("unterminated string constant", *line);
      return false;
    } else if (ch == '"') {
      AddToken(TokenString, token, include);
      return true;
    } else {
      token[len++] = ch;
    }
  }
  assert(false);
  return false;
}

///////////////////////////////////////////////////////////////////////////
// Read in a whitespace token
bool ConfigFile::LoadTokenSpace(FILE *file, int *line, int include) {
  int ch;
  int len;
  char token[1024];

  len = 0;
  memset(token, 0, sizeof(token));

  while (true) {
    ch = fgetc(file);

    if (ch == EOF) {
      AddToken(TokenSpace, token, include);
      return true;
    } else if (isblank(ch)) {
      token[len++] = ch;
    } else {
      AddToken(TokenSpace, token, include);
      ungetc(ch, file);
      return true;
    }
  }
  assert(false);
  return false;
}

///////////////////////////////////////////////////////////////////////////
// Save tokens to a file.
bool ConfigFile::SaveTokens(FILE *file) {
  int i;
  Token *token;

  for (i = 0; i < this->token_count; i++) {
    token = this->tokens + i;

    if (token->include > 0)
      continue;
    if (token->type == TokenString)
      fprintf(file, "\"%s\"", token->value);
    else
      fprintf(file, "%s", token->value);
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////
// Clear the token list
void ConfigFile::ClearTokens() {
  int i;
  Token *token;
  for (i = 0; i < this->token_count; i++) {
    token = this->tokens + i;
    free(token->value);
  }
  free(this->tokens);
  this->tokens = 0;
  this->token_size = 0;
  this->token_count = 0;
}

///////////////////////////////////////////////////////////////////////////
// Add a token to the token list
bool ConfigFile::AddToken(int type, const char *value, int include) {
  if (this->token_count >= this->token_size) {
    this->token_size += 1000;
    this->tokens = (Token *)realloc(this->tokens,
                                    this->token_size * sizeof(this->tokens[0]));
  }

  this->tokens[this->token_count].include = include;
  this->tokens[this->token_count].type = type;
  this->tokens[this->token_count].value = strdup(value);
  this->token_count++;

  return true;
}

///////////////////////////////////////////////////////////////////////////
// Set a token value in the token list
bool ConfigFile::SetTokenValue(int index, const char *value) {
  assert(index >= 0 && index < this->token_count);

  free(this->tokens[index].value);
  this->tokens[index].value = strdup(value);

  return true;
}

///////////////////////////////////////////////////////////////////////////
// Get the value of a token
const char *ConfigFile::GetTokenValue(int index) {
  assert(index >= 0 && index < this->token_count);

  return this->tokens[index].value;
}

///////////////////////////////////////////////////////////////////////////
// Dump the token list (for debugging).
void ConfigFile::DumpTokens() {
  int line;

  line = 1;
  printf("\n## begin tokens\n");
  printf("## %4d : ", line);
  for (int i = 0; i < this->token_count; i++) {
    if (this->tokens[i].value[0] == '\n')
      printf("[\\n]\n## %4d : %02d ", ++line, this->tokens[i].include);
    else
      printf("[%s] ", this->tokens[i].value);
  }
  printf("\n");
  printf("## end tokens\n");
}

///////////////////////////////////////////////////////////////////////////
// Parse tokens into sections and fields.
bool ConfigFile::ParseTokens() {
  int i;
  int section;
  int line;
  Token *token;

  ClearSections();
  ClearFields();

  // Add in the "global" section.
  section = AddSection(-1, "");
  line = 1;

  for (i = 0; i < this->token_count; i++) {
    token = this->tokens + i;

    switch (token->type) {
    case TokenWord:
      if (strcmp(token->value, "include") == 0) {
        if (!ParseTokenInclude(&i, &line))
          return false;
      } else if (strcmp(token->value, "define") == 0) {
        if (!ParseTokenDefine(&i, &line))
          return false;
      } else {
        if (!ParseTokenWord(section, &i, &line))
          return false;
      }
      break;
    case TokenComment:
      break;
    case TokenSpace:
      break;
    case TokenEOL:
      line++;
      break;
    default:
      PARSE_ERR("syntax error 1", line);
      return false;
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////
// Parse an include statement
bool ConfigFile::ParseTokenInclude(int *index, int *line) {
  int i;
  Token *token;

  for (i = *index + 1; i < this->token_count; i++) {
    token = this->tokens + i;

    switch (token->type) {
    case TokenString:
      break;
    case TokenSpace:
      break;
    case TokenEOL:
      *index = i;
      (*line)++;
      return true;
    default:
      PARSE_ERR("syntax error in include statement", *line);
    }
  }
  PARSE_ERR("incomplete include statement", *line);
  return false;
}

///////////////////////////////////////////////////////////////////////////
// Parse a macro definition
bool ConfigFile::ParseTokenDefine(int *index, int *line) {
  int i;
  int count;
  const char *macroname, *sectionname;
  int starttoken;
  Token *token;

  count = 0;
  macroname = NULL;
  sectionname = NULL;
  starttoken = -1;

  for (i = *index + 1; i < this->token_count; i++) {
    token = this->tokens + i;

    switch (token->type) {
    case TokenWord:
      if (count == 0) {
        if (macroname == NULL)
          macroname = GetTokenValue(i);
        else if (sectionname == NULL) {
          sectionname = GetTokenValue(i);
          starttoken = i;
        } else {
          PARSE_ERR("extra tokens in macro definition", *line);
          return false;
        }
      } else {
        if (macroname == NULL) {
          PARSE_ERR("missing name in macro definition", *line);
          return false;
        }
        if (sectionname == NULL) {
          PARSE_ERR("missing name in macro definition", *line);
          return false;
        }
      }
      break;
    case TokenOpenSection:
      count++;
      break;
    case TokenCloseSection:
      count--;
      if (count == 0) {
        AddMacro(macroname, sectionname, *line, starttoken, i);
        *index = i;
        return true;
      }
      if (count < 0) {
        PARSE_ERR("misplaced ')'", *line);
        return false;
      }
      break;
    default:
      break;
    }
  }
  PARSE_ERR("missing ')'", *line);
  return false;
}

///////////////////////////////////////////////////////////////////////////
// Parse something starting with a word; could be a section or an field.
bool ConfigFile::ParseTokenWord(int section, int *index, int *line) {
  int i;
  Token *token;

  for (i = *index + 1; i < this->token_count; i++) {
    token = this->tokens + i;

    switch (token->type) {
    case TokenComment:
      break;
    case TokenSpace:
      break;
    case TokenEOL:
      (*line)++;
      break;
    case TokenOpenSection:
      return ParseTokenSection(section, index, line);
    case TokenNum:
    case TokenString:
    case TokenOpenTuple:
    case TokenBool:
      return ParseTokenField(section, index, line);
    default:
      PARSE_ERR("syntax error 2", *line);
      return false;
    }
  }

  return false;
}

///////////////////////////////////////////////////////////////////////////
// Parse a section from the token list.
bool ConfigFile::ParseTokenSection(int section, int *index, int *line) {
  int i;
  int macro;
  int name;
  Token *token;

  name = *index;
  macro = LookupMacro(GetTokenValue(name));

  // If the section name is a macro...
  if (macro >= 0) {
    // This is a bit of a hack
    int nsection = this->section_count;
    int mindex = this->macros[macro].starttoken;
    int mline = this->macros[macro].line;
    if (!ParseTokenSection(section, &mindex, &mline))
      return false;
    section = nsection;

    for (i = *index + 1; i < this->token_count; i++) {
      token = this->tokens + i;

      switch (token->type) {
      case TokenOpenSection:
        break;
      case TokenWord:
        if (!ParseTokenWord(section, &i, line))
          return false;
        break;
      case TokenCloseSection:
        *index = i;
        return true;
      case TokenComment:
        break;
      case TokenSpace:
        break;
      case TokenEOL:
        (*line)++;
        break;
      default:
        PARSE_ERR("syntax error 3", *line);
        return false;
      }
    }
    PARSE_ERR("missing ')'", *line);
  }
  // If the section name is not a macro...
  else {
    for (i = *index + 1; i < this->token_count; i++) {
      token = this->tokens + i;

      switch (token->type) {
      case TokenOpenSection:
        section = AddSection(section, GetTokenValue(name));
        break;
      case TokenWord:
        if (!ParseTokenWord(section, &i, line))
          return false;
        break;
      case TokenCloseSection:
        *index = i;
        return true;
      case TokenComment:
        break;
      case TokenSpace:
        break;
      case TokenEOL:
        (*line)++;
        break;
      default:
        PARSE_ERR("syntax error 3", *line);
        return false;
      }
    }
    PARSE_ERR("missing ')'", *line);
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////
// Parse an field from the token list.
bool ConfigFile::ParseTokenField(int section, int *index, int *line) {
  int i, field;
  int name;
  Token *token;

  name = *index;

  for (i = *index + 1; i < this->token_count; i++) {
    token = this->tokens + i;

    switch (token->type) {
    case TokenNum:
      field = AddField(section, GetTokenValue(name), *line);
      AddFieldValue(field, 0, i);
      *index = i;
      return true;
    case TokenString:
      field = AddField(section, GetTokenValue(name), *line);
      AddFieldValue(field, 0, i);
      *index = i;
      return true;
    case TokenOpenTuple:
      field = AddField(section, GetTokenValue(name), *line);
      if (!ParseTokenTuple(section, field, &i, line))
        return false;
      *index = i;
      return true;
    case TokenBool:
      field = AddField(section, GetTokenValue(name), *line);
      AddFieldValue(field, 0, i);
      *index = i;
      return true;
    case TokenSpace:
      break;
    default:
      PARSE_ERR("syntax error 4", *line);
      return false;
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////
// Parse a tuple.
bool ConfigFile::ParseTokenTuple(int section, int field, int *index,
                                 int *line) {
  int i, count;
  Token *token;

  count = 0;

  for (i = *index + 1; i < this->token_count; i++) {
    token = this->tokens + i;

    switch (token->type) {
    case TokenNum:
      AddFieldValue(field, count++, i);
      *index = i;
      break;
    case TokenString:
      AddFieldValue(field, count++, i);
      *index = i;
      break;
    case TokenCloseTuple:
      *index = i;
      return true;
    case TokenSpace:
    case TokenEOL:
      break;
    default:
      PARSE_ERR("syntax error 5", *line);
      return false;
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////
// Clear the macro list
void ConfigFile::ClearMacros() {
  free(this->macros);
  this->macros = NULL;
  this->macro_size = 0;
  this->macro_count = 0;
}

///////////////////////////////////////////////////////////////////////////
// Add a macro
int ConfigFile::AddMacro(const char *macroname, const char *sectionname,
                         int line, int starttoken, int endtoken) {
  if (this->macro_count >= this->macro_size) {
    this->macro_size += 100;
    this->macros = (CMacro *)realloc(this->macros, this->macro_size *
                                                       sizeof(this->macros[0]));
  }

  int macro = this->macro_count;
  this->macros[macro].macroname = macroname;
  this->macros[macro].sectionname = sectionname;
  this->macros[macro].line = line;
  this->macros[macro].starttoken = starttoken;
  this->macros[macro].endtoken = endtoken;
  this->macro_count++;

  return macro;
}

///////////////////////////////////////////////////////////////////////////
// Lookup a macro by name
// Returns -1 if there is no macro with this name.
int ConfigFile::LookupMacro(const char *macroname) {
  int i;
  CMacro *macro;

  for (i = 0; i < this->macro_count; i++) {
    macro = this->macros + i;
    if (strcmp(macro->macroname, macroname) == 0)
      return i;
  }
  return -1;
}

///////////////////////////////////////////////////////////////////////////
// Dump the macro list for debugging
void ConfigFile::DumpMacros() {
  printf("\n## begin macros\n");
  for (int i = 0; i < this->macro_count; i++) {
    CMacro *macro = this->macros + i;

    printf("## [%s][%s]", macro->macroname, macro->sectionname);
    for (int j = macro->starttoken; j <= macro->endtoken; j++) {
      if (this->tokens[j].type == TokenEOL)
        printf("[\\n]");
      else
        printf("[%s]", GetTokenValue(j));
    }
    printf("\n");
  }
  printf("## end macros\n");
}

///////////////////////////////////////////////////////////////////////////
// Clear the section list
void ConfigFile::ClearSections() {
  free(this->sections);
  this->sections = NULL;
  this->section_size = 0;
  this->section_count = 0;
}

///////////////////////////////////////////////////////////////////////////
// Add a section
int ConfigFile::AddSection(int parent, const char *type) {
  if (this->section_count >= this->section_size) {
    // printf("AddSection here \r\n");
    this->section_size += 100;
    this->sections = (Section *)realloc(
        this->sections, this->section_size * sizeof(this->sections[0]));
  }

  int section = this->section_count;
  this->sections[section].parent = parent;
  this->sections[section].type = type;
  this->section_count++;

  return section;
}

///////////////////////////////////////////////////////////////////////////
// Get the number of sections
int ConfigFile::GetSectionCount() { return this->section_count; }

///////////////////////////////////////////////////////////////////////////
// Get a section's parent section
int ConfigFile::GetSectionParent(int section) {
  if (section < 0 || section >= this->section_count)
    return -1;
  return this->sections[section].parent;
}

///////////////////////////////////////////////////////////////////////////
// Get a section (returns the section type value)
const char *ConfigFile::GetSectionType(int section) {
  if (section < 0 || section >= this->section_count)
    return NULL;
  return this->sections[section].type;
}

///////////////////////////////////////////////////////////////////////////
// Lookup a section number by type name
// Returns -1 if there is no section with this type
int ConfigFile::LookupSection(const char *type) {
  for (int section = 0; section < GetSectionCount(); section++) {
    if (strcmp(GetSectionType(section), type) == 0)
      return section;
  }
  return -1;
}

///////////////////////////////////////////////////////////////////////////
// Dump the section list for debugging
void ConfigFile::DumpSections() {
  printf("\n## begin sections section_count=%d\n", this->section_count);
  for (int i = 0; i < this->section_count; i++) {
    Section *section = this->sections + i;

    printf("## [%d][%d]", i, section->parent);
    printf("[%s]\n", section->type);
  }
  printf("## end sections\n");
}

///////////////////////////////////////////////////////////////////////////
// Clear the field list
void ConfigFile::ClearFields() {
  int i;
  Field *field;

  for (i = 0; i < this->field_count; i++) {
    field = this->fields + i;
    free(field->values);
    free(field->useds);
    field->values = NULL;
    field->useds = NULL;
  }
  free(this->fields);
  this->fields = NULL;
  this->field_size = 0;
  this->field_count = 0;
}

///////////////////////////////////////////////////////////////////////////
// Add an field
int ConfigFile::AddField(int section, const char *name, int line) {
  int i;
  Field *field;

  // See if this field already exists; if it does, we dont need to
  // add it again.
  // printf("addfiled 000\r\n");
  // printf("AddField field_count = %d\r\n", this->field_count);
  for (i = 0; i < this->field_count; i++) {
    field = this->fields + i;
    // printf("this_section = %d section=%d\r\n", field->section, section);
    if (field->section != section) {
      continue;
    }

    if (strcmp(field->name, name) == 0) {
      // printf("field->name: %s, name: %s\r\n", field->name, name);
      return i;
    }
  }

  // printf("addfiled 111\r\n");
  // Expand field array if necessary.
  if (i >= this->field_size) {
    this->field_size += 100;
    this->fields = (Field *)realloc(this->fields,
                                    this->field_size * sizeof(this->fields[0]));
  }

  field = this->fields + i;
  memset(field, 0, sizeof(Field));
  field->section = section;
  field->name = name;
  field->value_count = 0;
  field->values = NULL;
  field->useds = NULL;
  field->line = line;

  this->field_count++;
  // printf("addfiled 222\r\n");
  return i;
}

///////////////////////////////////////////////////////////////////////////
// Add a field value
void ConfigFile::AddFieldValue(int field, int index, int value_token) {
  assert(field >= 0);
  Field *pfield = this->fields + field;

  // Expand the array if it's too small
  if (index >= pfield->value_count) {
    pfield->value_count = index + 1;
    pfield->values =
        (int *)realloc(pfield->values, pfield->value_count * sizeof(int));
    pfield->useds =
        (bool *)realloc(pfield->useds, pfield->value_count * sizeof(bool));
    pfield->useds[pfield->value_count - 1] = false;
  }

  // Set the relevant value
  pfield->values[index] = value_token;
  pfield->useds[index] = false;
}

///////////////////////////////////////////////////////////////////////////
// Get a field
int ConfigFile::GetField(int section, const char *name) {
  // Find first instance of field
  for (int i = 0; i < this->field_count; i++) {
    Field *field = this->fields + i;
    if (field->section != section)
      continue;
    if (strcmp(field->name, name) == 0)
      return i;
  }
  return -1;
}

///////////////////////////////////////////////////////////////////////////
// Get the number of elements for this field
int ConfigFile::GetFieldValueCount(int field) {
  Field *pfield = this->fields + field;
  return pfield->value_count;
}

///////////////////////////////////////////////////////////////////////////
// Set the value of an field
void ConfigFile::SetFieldValue(int field, int index, const char *value) {
  // assert(field >= 0 && field < this->field_count);
  Field *pfield = this->fields + field;
  assert(index >= 0 && index < pfield->value_count);

  // Set the relevant value
  SetTokenValue(pfield->values[index], value);
}

///////////////////////////////////////////////////////////////////////////
// Get the value of an field
const char *ConfigFile::GetFieldValue(int field, int index, bool flag_used) {
  assert(field >= 0);
  Field *pfield = this->fields + field;

  if (index >= pfield->value_count)
    return NULL;

  if (flag_used)
    pfield->useds[index] = true;

  return GetTokenValue(pfield->values[index]);
}

///////////////////////////////////////////////////////////////////////////
// Dump the field list for debugging
void ConfigFile::DumpFields() {
  printf("\n## begin fields field_count=%d section=%d this->sectionsPt=%p\n",
         this->field_count, this->section_count, (void *)this->sections);

  for (int i = 0; i < this->field_count; i++) {
    Field *field = this->fields + i;
    Section *section = this->sections + field->section;
    printf("field->section = %d\r\n", field->section);

    printf("## [%d]", field->section);
    printf("[%s]", section->type);
    printf("[%s]\n", field->name);
    for (int j = 0; j < field->value_count; j++)
      printf("[%s]\n", GetTokenValue(field->values[j]));
    printf("\n");
  }
  printf("## end fields\n");
}

///////////////////////////////////////////////////////////////////////////
// Read a string
const char *ConfigFile::ReadString(int section, const char *name,
                                   const char *value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;
  return GetFieldValue(field, 0);
}

const char *ConfigFile::ReadString(const char *name, const char *value) {
  return ReadString(0, name, value);
}

///////////////////////////////////////////////////////////////////////////
// Write a string
void ConfigFile::WriteString(int section, const char *name, const char *value) {
  int field = GetField(section, name);
  if (field < 0)
    return;
  SetFieldValue(field, 0, value);
}
void ConfigFile::WriteString(const char *name, const char *value) {
  WriteString(0, name, value);
}

///////////////////////////////////////////////////////////////////////////
// Read an int
int ConfigFile::ReadInt(int section, const char *name, int value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;
  return atoi(GetFieldValue(field, 0));
}

int ConfigFile::ReadInt(const char *name, int value) {
  return ReadInt(0, name, value);
}

///////////////////////////////////////////////////////////////////////////
// Write an int
void ConfigFile::WriteInt(int section, const char *name, int value) {
  char default_str[64];
  snprintf(default_str, sizeof(default_str), "%d", value);
  WriteString(section, name, default_str);
}

void ConfigFile::WriteInt(const char *name, int value) {
  WriteInt(0, name, value);
}

///////////////////////////////////////////////////////////////////////////
// Read a boolean
bool ConfigFile::ReadBool(int section, const char *name, bool value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;
  const char *s = GetFieldValue(field, 0);
  if (strcmp(s, "yes") == 0 || strcmp(s, "true") == 0 || strcmp(s, "1") == 0)
    return true;
  else if (strcmp(s, "no") == 0 || strcmp(s, "false") == 0 ||
           strcmp(s, "0") == 0)
    return false;
  else {
    printf("AnyEye: Warning: error in config file section %d field %s: Invalid "
           "boolean value.",
           section, name);
    return value;
  }
}
bool ConfigFile::ReadBool(const char *name, bool value) {
  return ReadBool(0, name, value);
}

///////////////////////////////////////////////////////////////////////////
// Write a boolean
void ConfigFile::WriteBool(int section, const char *name, bool value) {
  char str[4];
  snprintf(str, sizeof(str), "%s", value ? "yes" : "no");
  WriteString(section, name, str);
}

void ConfigFile::WriteBool(const char *name, bool value) {
  WriteBool(0, name, value);
}

///////////////////////////////////////////////////////////////////////////
// Write a boolean compatibly
void ConfigFile::WriteBool_Compat(int section, const char *name, bool value) {
  char str[2];
  snprintf(str, sizeof(str), "%s", value ? "1" : "0");
  WriteString(section, name, str);
}
void ConfigFile::WriteBool_Compat(const char *name, bool value) {
  WriteBool_Compat(0, name, value);
}

///////////////////////////////////////////////////////////////////////////
// Read a float
double ConfigFile::ReadFloat(int section, const char *name, double value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;
  return atof(GetFieldValue(field, 0));
}
double ConfigFile::ReadFloat(const char *name, double value) {
  return ReadFloat(0, name, value);
}

// Write an float
void ConfigFile::WriteFloat(int section, const char *name, double value) {
  char default_str[64];
  snprintf(default_str, sizeof(default_str), "%.3f", value);
  WriteString(section, name, default_str);
}

void ConfigFile::WriteFloat(const char *name, double value) {
  WriteFloat(0, name, value);
}

///////////////////////////////////////////////////////////////////////////
// Read a length (includes unit conversion)
double ConfigFile::ReadLength(int section, const char *name, double value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;
  return atof(GetFieldValue(field, 0)) * this->unit_length;
}
double ConfigFile::ReadLength(const char *name, double value) {
  return ReadLength(0, name, value);
}

///////////////////////////////////////////////////////////////////////////
// Write a length (includes units conversion)
void ConfigFile::WriteLength(int section, const char *name, double value) {
  char default_str[64];
  snprintf(default_str, sizeof(default_str), "%.3f", value / this->unit_length);
  WriteString(section, name, default_str);
}
void ConfigFile::WriteLength(const char *name, double value) {
  WriteLength(0, name, value);
}

///////////////////////////////////////////////////////////////////////////
// Read an angle (includes unit conversion)
double ConfigFile::ReadAngle(int section, const char *name, double value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;
  return atof(GetFieldValue(field, 0)) * this->unit_angle;
}

///////////////////////////////////////////////////////////////////////////
// Read a color (included text -> RGB conversion).
// We look up the color in one of the common color databases.
uint32_t ConfigFile::ReadColor(int section, const char *name, uint32_t value) {
  int field;
  const char *color;

  field = GetField(section, name);
  if (field < 0)
    return value;
  color = GetFieldValue(field, 0);

  return LookupColor(color);
}

///////////////////////////////////////////////////////////////////////////
// Read a file name.
// Always returns an absolute path.
// If the filename is entered as a relative path, we prepend
// the world files path to it.
const char *ConfigFile::ReadFilename(int section, const char *name,
                                     const char *value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;

  const char *filename = GetFieldValue(field, 0);

  if (filename[0] == '/' || filename[0] == '~')
    return filename;

  else if (this->filename[0] == '/' || this->filename[0] == '~') {
    // Note that dirname() modifies the contents, so
    // we need to make a copy of the filename.
    // There's no bounds-checking, but what the heck.
    char *tmp = strdup(this->filename.c_str());
    char *fullpath = (char *)malloc(MAX_FILENAME_SIZE);
    memset(fullpath, 0, MAX_FILENAME_SIZE);
    strcat(fullpath, dirname(tmp));
    strcat(fullpath, "/");
    strcat(fullpath, filename);
    assert(strlen(fullpath) + 1 < MAX_FILENAME_SIZE);

    SetFieldValue(field, 0, fullpath);

    free(fullpath);
    free(tmp);
  } else {
    // Prepend the path
    // Note that dirname() modifies the contents, so
    // we need to make a copy of the filename.
    // There's no bounds-checking, but what the heck.
    char *tmp = strdup(this->filename.c_str());
    char *fullpath = (char *)malloc(MAX_FILENAME_SIZE);
    char *ret = getcwd(fullpath, MAX_FILENAME_SIZE);
    if (ret == NULL) {
      printf("Failed to get working directory\r\n");
      assert(false);
      // TODO: handle this error properly
    }
    strcat(fullpath, "/");
    strcat(fullpath, dirname(tmp));
    strcat(fullpath, "/");
    strcat(fullpath, filename);
    assert(strlen(fullpath) + 1 < MAX_FILENAME_SIZE);

    SetFieldValue(field, 0, fullpath);

    free(fullpath);
    free(tmp);
  }

  filename = GetFieldValue(field, 0);
  assert(filename[0] == '/' || filename[0] == '~');

  return filename;
}

///////////////////////////////////////////////////////////////////////////
// Get the number of values in a tuple
int ConfigFile::GetTupleCount(int section, const char *name) {
  int field = GetField(section, name);
  if (field < 0)
    return 0;
  return GetFieldValueCount(field);
}

///////////////////////////////////////////////////////////////////////////
// Read a string from a tuple
const char *ConfigFile::ReadTupleString(int section, const char *name,
                                        int index, const char *value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;
  const char *svalue = GetFieldValue(field, index);
  if (!svalue)
    return value;
  return svalue;
}

///////////////////////////////////////////////////////////////////////////
// Write a string to a tuple
void ConfigFile::WriteTupleString(int section, const char *name, int index,
                                  const char *value) {
  int field = GetField(section, name);
  /* TODO
  if (field < 0)
    field = InsertField(section, name);
  */
  SetFieldValue(field, index, value);
}

///////////////////////////////////////////////////////////////////////////
// Read a int from a tuple
int ConfigFile::ReadTupleInt(int section, const char *name, int index,
                             int value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;
  const char *svalue = GetFieldValue(field, index);
  if (!svalue)
    return value;
  return atoi(svalue);
}

///////////////////////////////////////////////////////////////////////////
// Write a int to a tuple
void ConfigFile::WriteTupleInt(int section, const char *name, int index,
                               int value) {
  char default_str[64];
  snprintf(default_str, sizeof(default_str), "%d", value);
  WriteTupleString(section, name, index, default_str);
}

///////////////////////////////////////////////////////////////////////////
// Read a float from a tuple
double ConfigFile::ReadTupleFloat(int section, const char *name, int index,
                                  double value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;
  const char *svalue = GetFieldValue(field, index);
  if (!svalue)
    return value;
  return atof(svalue);
}

///////////////////////////////////////////////////////////////////////////
// Write a float to a tuple
void ConfigFile::WriteTupleFloat(int section, const char *name, int index,
                                 double value) {
  char default_str[64];
  snprintf(default_str, sizeof(default_str), "%.3f", value);
  WriteTupleString(section, name, index, default_str);
}

///////////////////////////////////////////////////////////////////////////
// Read a length from a tuple (includes unit conversion)
double ConfigFile::ReadTupleLength(int section, const char *name, int index,
                                   double value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;
  const char *svalue = GetFieldValue(field, index);
  if (!svalue)
    return value;
  return atof(svalue) * this->unit_length;
}

///////////////////////////////////////////////////////////////////////////
// Write a length to a tuple (includes unit conversion)
void ConfigFile::WriteTupleLength(int section, const char *name, int index,
                                  double value) {
  char default_str[64];
  snprintf(default_str, sizeof(default_str), "%.3f", value / this->unit_length);
  WriteTupleString(section, name, index, default_str);
}

///////////////////////////////////////////////////////////////////////////
// Read an angle from a tuple (includes unit conversion)
double ConfigFile::ReadTupleAngle(int section, const char *name, int index,
                                  double value) {
  int field = GetField(section, name);
  if (field < 0)
    return value;
  const char *svalue = GetFieldValue(field, index);
  if (!svalue)
    return value;
  return atof(svalue) * this->unit_angle;
}

///////////////////////////////////////////////////////////////////////////
// Write an angle to a tuple (includes unit conversion)
void ConfigFile::WriteTupleAngle(int section, const char *name, int index,
                                 double value) {
  char default_str[64];
  snprintf(default_str, sizeof(default_str), "%.3f", value / this->unit_angle);
  WriteTupleString(section, name, index, default_str);
}

///////////////////////////////////////////////////////////////////////////
// Read a color (included text -> RGB conversion).
// We look up the color in one of the common color databases.
uint32_t ConfigFile::ReadTupleColor(int section, const char *name, int index,
                                    uint32_t value) {
  int field;
  const char *color;

  field = GetField(section, name);
  if (field < 0)
    return value;
  color = GetFieldValue(field, index);
  if (!color)
    return value;

  return LookupColor(color);
}

///////////////////////////////////////////////////////////////////////////
// Look up the color in a data based (transform color name -> color value).
uint32_t ConfigFile::LookupColor(const char *name) {
  FILE *file = NULL;

  for (int i = 0; COLOR_DATABASE[i] != NULL; ++i) {
    file = fopen(COLOR_DATABASE[i], "r");
    if (file)
      break;
  }
  if (!file) {
    printf("unable to open color database: tried the following files\r\n");
    for (int i = 0; COLOR_DATABASE[i] != NULL; ++i) {
      printf("\t: %s\r\n", COLOR_DATABASE[i]);
    }
    fclose(file);
    return 0xFFFFFF;
  }

  while (true) {
    char line[1024];
    if (!fgets(line, sizeof(line), file))
      break;

    // it's a macro or comment line - ignore the line
    if (line[0] == '!' || line[0] == '#' || line[0] == '%')
      continue;

    // Trim the trailing space
    while (strchr(" \t\n", line[strlen(line) - 1]))
      line[strlen(line) - 1] = 0;

    // Read the color
    int r, g, b;
    int chars_matched = 0;
    sscanf(line, "%d %d %d %n", &r, &g, &b, &chars_matched);

    // Read the name
    char *nname = line + chars_matched;

    // If the name matches
    if (strcmp(nname, name) == 0) {
      fclose(file);
      return ((r << 16) | (g << 8) | b);
    }
  }
  printf("unable to find color [%s]; using default (red)\r\n", name);
  fclose(file);
  return 0xFF0000;
}

char *dirname(char *path) {
  static const char dot[] = ".";
  char *last_slash;

  /* Find last '/'.  */
  last_slash = path != NULL ? strrchr(path, '/') : NULL;

  if (last_slash == path)
    /* The last slash is the first character in the string.  We have to
       return "/".  */
    ++last_slash;
  else if (last_slash != NULL && last_slash[1] == '\0')
    /* The '/' is the last character, we have to look further.  */
    last_slash = (char *)memchr(path, last_slash - path, '/');

  if (last_slash != NULL)
    /* Terminate the path.  */
    last_slash[0] = '\0';
  else
    /* This assignment is ill-designed but the XPG specs require to
       return a string containing "." in any case no directory part is
       found and so a static and constant string is required.  */
    path = (char *)dot;

  return path;
}