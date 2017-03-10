/******************************************************************************
 * Copyright 2017 Samuel Bachmann                                             *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions are met:*
 *                                                                            *
 * 1. Redistributions of source code must retain the above copyright notice,  *
 * this list of conditions and the following disclaimer.                      *
 *                                                                            *
 * 2. Redistributions in binary form must reproduce the above copyright       *
 * notice, this list of conditions and the following disclaimer in the        *
 * documentation and/or other materials provided with the distribution.       *
 *                                                                            *
 * 3. Neither the name of the copyright holder nor the names of its           *
 * contributors may be used to endorse or promote products derived from this  *
 * software without specific prior written permission.                        *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE  *
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR        *
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF       *
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS   *
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN    *
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)    *
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF     *
 * THE POSSIBILITY OF SUCH DAMAGE.                                            *
 *                                                                            *
 * Author: Samuel Bachmann <samuel.bachmann@gmail.com>                        *
 ******************************************************************************/

#include "rqt_free_gait/CircularBuffer.h"

namespace rqt_free_gait {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

CircularBuffer::CircularBuffer(unsigned int length) : length_(length) {

}

CircularBuffer::~CircularBuffer() {

}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void CircularBuffer::push_back(description_t description) {
  QRegExp regExp;
  regExp.setPattern("\\n[-]{3,10}\\n");
  description.message.replace(regExp, "<hr>");
  regExp.setPattern("\\n");
  description.message.replace(regExp, "<br>");
  descriptions_.push_back(description);
  if (descriptions_.size() > length_) {
    descriptions_.pop_front();
    index_--;
    index_ = std::max(index_, 0);
  }
}

unsigned long CircularBuffer::size() {
  return descriptions_.size();
}

int CircularBuffer::moveIndex(int steps) {
  index_ += steps;
  if (index_ < 0) {
    index_ = 0;
  } else if (index_ > size() - 1) {
    index_ = (int)size() - 1;
  }
  return index_;
}

description_t CircularBuffer::back() {
  return descriptions_.back();
}

description_t CircularBuffer::front() {
  return descriptions_.front();
}

description_t CircularBuffer::current() {
  return descriptions_[index_];
}

QString CircularBuffer::backQString() {
  return composeDescription(descriptions_.back());
}

QString CircularBuffer::frontQString() {
  return composeDescription(descriptions_.front());
}

QString CircularBuffer::currentQString() {
  return composeDescription(descriptions_[index_]);
}

int CircularBuffer::index() {
  return index_;
}

int CircularBuffer::moveIndexBack() {
  index_ = (int)size() - 1;
  return index_;
}

int CircularBuffer::moveIndexFront() {
  index_ = 0;
  return index_;
}

void CircularBuffer::clear() {
  index_ = 0;
  descriptions_.clear();
}

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

QString CircularBuffer::composeDescription(description_t description) {
  QString str = "<html><b>" + description.timestamp + "</b>" + "<hr>" +
      description.message + "</html>";
  return str;
}

} // namespace
