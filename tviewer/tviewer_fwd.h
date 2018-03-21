/******************************************************************************
 * Copyright (c) 2014 Sergey Alexandrov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#pragma once

/** \file tviewer_fwd.h
  * Forward declarations and typedefs */

#include <memory>

namespace tviewer
{

  class TViewerImpl;
  class TViewerDummy;
  class TViewerInterface;
  class VisualizationObject;
  class KeyboardListener;

  /// Shared pointer to an object that implements TViewer interface.
  using TViewerPtr = std::shared_ptr<TViewerInterface>;

  /// Shared pointer to a visualization object.
  using VisualizationObjectPtr = std::shared_ptr<VisualizationObject>;

  /// Shared pointer to a keyboard listener.
  using KeyboardListenerPtr = std::shared_ptr<KeyboardListener>;

  /// RGBA color.
  using Color = uint32_t;

}

