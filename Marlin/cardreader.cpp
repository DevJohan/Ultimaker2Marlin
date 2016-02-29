#include "Marlin.h"
#include "cardreader.h"
#include "UltiLCD2.h"
#include "ultralcd.h"
#include "stepper.h"
#include "temperature.h"
#include "language.h"
#include "../CommunicationsBridge/printer_to_remote.h"

#ifdef SDSUPPORT



CardReader::CardReader()
{
   filesize = 0;
   sdpos = 0;
   sdprinting = false;
   pause = false;
   cardOK = false;
   saving = false;
   logging = false;
   autostart_atmillis=0;
   workDirDepth = 0;
   memset(workDirParents, 0, sizeof(workDirParents));

   autostart_stilltocheck=true; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.
   lastnr=0;
  //power to SD reader
  #if SDPOWER > -1
    SET_OUTPUT(SDPOWER);
    WRITE(SDPOWER,HIGH);
  #endif //SDPOWER

  autostart_atmillis=millis()+5000;
}

char *createFilename(char *buffer,const dir_t &p) //buffer>12characters
{
  char *pos=buffer;
  for (uint8_t i = 0; i < 11; i++)
  {
    if (p.name[i] == ' ')continue;
    if (i == 8)
    {
      *pos++='.';
    }
    *pos++=p.name[i];
  }
  *pos++=0;
  return buffer;
}


void  CardReader::lsDive(const char *prepend,SdFile parent)
{
  dir_t p;
 uint8_t cnt=0;

  while (parent.readDir(p, longFilename) > 0)
  {
    if( DIR_IS_SUBDIR(&p) && lsAction!=LS_Count && lsAction!=LS_GetFilename) // hence LS_SerialPrint
    {

      char path[13*2];
      char lfilename[13];
      createFilename(lfilename,p);

      path[0]=0;
      if(strlen(prepend)==0) //avoid leading / if already in prepend
      {
       strcat(path,"/");
      }
      strcat(path,prepend);
      strcat(path,lfilename);
      strcat(path,"/");

      //Serial.print(path);

      SdFile dir;
      if(!dir.open(parent,lfilename, O_READ))
      {
        if(lsAction==LS_SerialPrint)
        {
        	report_open_subdir_failed(lfilename);
        }
      }
      lsDive(path,dir);
      //close done automatically by destructor of SdFile


    }
    else
    {
      if (p.name[0] == DIR_NAME_FREE) break;
      if (p.name[0] == DIR_NAME_DELETED || p.name[0] == '.'|| p.name[0] == '_') continue;
      if (longFilename[0] != '\0' &&
          (longFilename[0] == '.' || longFilename[0] == '_')) continue;
      if ( p.name[0] == '.')
      {
        if ( p.name[1] != '.')
        continue;
      }

      if (!DIR_IS_FILE_OR_SUBDIR(&p)) continue;
      filenameIsDir=DIR_IS_SUBDIR(&p);


      if(!filenameIsDir)
      {
        if(p.name[8]!='G') continue;
        if(p.name[9]=='~') continue;
      }
      //if(cnt++!=nr) continue;
      createFilename(filename,p);
      if(lsAction==LS_SerialPrint)
      {
				report_sd_list_filename(prepend);
      }
      else if(lsAction==LS_Count)
      {
        nrFiles++;
      }
      else if(lsAction==LS_GetFilename)
      {
        if(cnt==nrFiles)
          return;
        cnt++;

      }
    }
  }
}

void CardReader::ls()
{
  lsAction=LS_SerialPrint;
  if(lsAction==LS_Count)
  nrFiles=0;

  root.rewind();
  lsDive("",root);
}



void CardReader::initsd()
{
  cardOK = false;
  if(root.isOpen())
    root.close();
#ifdef SDSLOW
  if (!card.init(SPI_HALF_SPEED,SDSS))
#else
  if (!card.init(SPI_FULL_SPEED,SDSS))
#endif
  {
    //if (!card.init(SPI_HALF_SPEED,SDSS))
		report_sd_init_card_failed();
  }
  else if (!volume.init(&card))
  {
		report_sd_init_volume_failed();
  }
  else if (!root.openRoot(&volume))
  {
		report_sd_open_root_failed();
  }
  else
  {
    cardOK = true;
    report_sd_init_card_ok();
  }
  workDir=root;
  curDir=&root;
  /*
  if(!workDir.openRoot(&volume))
  {
    SERIAL_ECHOLNPGM(MSG_SD_WORKDIR_FAIL);
  }
  */

}

void CardReader::setroot()
{
  /*if(!workDir.openRoot(&volume))
  {
    SERIAL_ECHOLNPGM(MSG_SD_WORKDIR_FAIL);
  }*/
  workDir=root;

  curDir=&workDir;
}
void CardReader::release()
{
  sdprinting = false;
  pause = false;
  cardOK = false;
}

void CardReader::startFileprint()
{
  if(cardOK)
  {
    sdprinting = true;
    pause = false;
  }
}

void CardReader::pauseSDPrint()
{
  if(sdprinting)
  {
    pause = true;
  }
}


void CardReader::openLogFile(const char* name)
{
  logging = true;
  openFile(name, false);
}


void CardReader::openFile(const char* name,bool read)
{
  if(!cardOK)
    return;
  file.close();
  sdprinting = false;
  pause = false;


  SdFile myDir;
  curDir=&root;
  const char *fname=name;

  char *dirname_start,*dirname_end;
  if(name[0]=='/')
  {
    dirname_start=strchr(name,'/')+1;
    while(dirname_start>(char*)1)
    {
      dirname_end=strchr(dirname_start,'/');
      //SERIAL_ECHO("start:");SERIAL_ECHOLN((int)(dirname_start-name));
      //SERIAL_ECHO("end  :");SERIAL_ECHOLN((int)(dirname_end-name));
      if(dirname_end>0 && dirname_end>dirname_start)
      {
        char subdirname[13];
        strncpy(subdirname, dirname_start, dirname_end-dirname_start);
        subdirname[dirname_end-dirname_start]=0;
   report_sd_subdir_name(subdirname);
        if(!myDir.open(curDir,subdirname,O_READ))
        {
        	report_sd_open_subdir_file_failed(subdirname);
          return;
        }
        else
        {
          //SERIAL_ECHOLN("dive ok");
        }

        curDir=&myDir;
        dirname_start=dirname_end+1;
      }
      else // the reminder after all /fsa/fdsa/ is the filename
      {
        fname=dirname_start;
        //SERIAL_ECHOLN("remaider");
        //SERIAL_ECHOLN(fname);
        break;
      }

    }
  }
  else //relative path
  {
    curDir=&workDir;
  }
  if(read)
  {
    if (file.open(curDir, fname, O_READ))
    {
      filesize = file.fileSize();
			report_sd_file_opened_for_reading(fname);
      sdpos = 0;

			report_sd_file_selected();
      lcd_setstatus(fname);
    }
    else
    {
			report_sd_open_file_failed(fname, read);
    }
  }
  else
  { //write
    if (!file.open(curDir, fname, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))
    {
		report_sd_open_file_failed(fname, read);
    }
    else
    {
      saving = true;
			report_sd_file_opened_for_writing(name);
      lcd_setstatus(fname);
    }
  }

}

void CardReader::removeFile(const char* name)
{
  if(!cardOK)
    return;
  file.close();
  sdprinting = false;
  pause = false;


  SdFile myDir;
  curDir=&root;
  const char *fname=name;

  char *dirname_start,*dirname_end;
  if(name[0]=='/')
  {
    dirname_start=strchr(name,'/')+1;
    while(dirname_start>0)
    {
      dirname_end=strchr(dirname_start,'/');
      //SERIAL_ECHO("start:");SERIAL_ECHOLN((int)(dirname_start-name));
      //SERIAL_ECHO("end  :");SERIAL_ECHOLN((int)(dirname_end-name));
      if(dirname_end>0 && dirname_end>dirname_start)
      {
        char subdirname[13];
        strncpy(subdirname, dirname_start, dirname_end-dirname_start);
        subdirname[dirname_end-dirname_start]=0;
  report_sd_subdir_name(subdirname);
        if(!myDir.open(curDir,subdirname,O_READ))
        {
  report_sd_open_subdir_file_failed(subdirname);
          return;
        }
        else
        {
          //SERIAL_ECHOLN("dive ok");
        }

        curDir=&myDir;
        dirname_start=dirname_end+1;
      }
      else // the reminder after all /fsa/fdsa/ is the filename
      {
        fname=dirname_start;
        //SERIAL_ECHOLN("remaider");
        //SERIAL_ECHOLN(fname);
        break;
      }

    }
  }
  else //relative path
  {
    curDir=&workDir;
  }
    if (file.remove(curDir, fname))
    {
		report_sd_file_removed(fname);
      sdpos = 0;
    }
    else
    {
		report_sd_remove_file_failed(fname);
    }

}


void CardReader::getStatus()
{
  if(cardOK){
		report_sd_printing_file_status(sdpos,filesize);
  }
  else{
		report_sd_not_printing();
  }
  if (card.errorCode())
  {
		report_sd_card_error();
  }
}


void CardReader::write_command(char *buf)
{
  char* begin = buf;
  char* npos = 0;
  char* end = buf + strlen(buf) - 1;

  file.writeError = false;
  if((npos = strchr(buf, 'N')) != NULL)
  {
    begin = strchr(npos, ' ') + 1;
    end = strchr(npos, '*') - 1;
  }
  end[1] = '\r';
  end[2] = '\n';
  end[3] = '\0';
  file.write(begin);
  if (file.writeError)
  {
		report_error_write_to_file();
  }
}

bool CardReader::write_string(char* buffer)
{
    file.write(buffer);
    return file.writeError;
}


void CardReader::checkautostart(bool force)
{
  if(!force)
  {
    if(!autostart_stilltocheck)
      return;
    if(autostart_atmillis<millis())
      return;
  }
  autostart_stilltocheck=false;
  if(!cardOK)
  {
    initsd();
    if(!cardOK) //fail
      return;
  }

  char autoname[30];
  sprintf_P(autoname, PSTR("auto%i.g"), lastnr);
  for(int8_t i=0;i<(int8_t)strlen(autoname);i++)
    autoname[i]=tolower(autoname[i]);
  dir_t p;

  root.rewind();

  bool found=false;
  while (root.readDir(p, NULL) > 0)
  {
    for(int8_t i=0;i<(int8_t)strlen((char*)p.name);i++)
    p.name[i]=tolower(p.name[i]);
    //Serial.print((char*)p.name);
    //Serial.print(" ");
    //Serial.println(autoname);
    if(p.name[9]!='~') //skip safety copies
    if(strncmp((char*)p.name,autoname,5)==0)
    {
      char cmd[30];

      sprintf_P(cmd, PSTR("M23 %s"), autoname);
      enquecommand(cmd);
      enquecommand_P(PSTR("M24"));
      found=true;
    }
  }
  if(!found)
    lastnr=-1;
  else
    lastnr++;
  clearError();
}

void CardReader::closefile()
{
  file.sync();
  file.close();
  saving = false;
  logging = false;
}

void CardReader::getfilename(const uint8_t nr)
{
  curDir=&workDir;
  lsAction=LS_GetFilename;
  nrFiles=nr;
  curDir->rewind();
  lsDive("",*curDir);

}

uint16_t CardReader::getnrfilenames()
{
  curDir=&workDir;
  lsAction=LS_Count;
  nrFiles=0;
  curDir->rewind();
  lsDive("",*curDir);
  //SERIAL_ECHOLN(nrFiles);
  return nrFiles;
}


void CardReader::chdir(const char * relpath)
{
  SdFile newfile;
  SdFile *parent=&root;

  if(workDir.isOpen())
    parent=&workDir;

  if(!newfile.open(*parent,relpath, O_READ))
  {
		report_cant_enter_subdir(relpath);
  }
  else
  {
    if (workDirDepth < MAX_DIR_DEPTH) {
      for (int d = ++workDirDepth; d--;)
        workDirParents[d+1] = workDirParents[d];
      workDirParents[0]=*parent;
    }
    workDir=newfile;
  }
}

void CardReader::updir()
{
  if(workDirDepth > 0)
  {
    --workDirDepth;
    workDir = workDirParents[0];
    for (uint8_t d = 0; d < workDirDepth; d++)
      workDirParents[d] = workDirParents[d+1];
  }
}


void CardReader::printingHasFinished()
{
    st_synchronize();
    quickStop();
    file.close();
    sdprinting = false;
    pause = false;
    if(SD_FINISHED_STEPPERRELEASE)
    {
        //finishAndDisableSteppers();
        enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    }
    autotempShutdown();
}
#endif //SDSUPPORT
