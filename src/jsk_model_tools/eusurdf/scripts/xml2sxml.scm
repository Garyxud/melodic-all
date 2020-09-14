#!/usr/bin/env gosh

(use sxml.ssax)

(define (main args)
  (call-with-input-file
      (cadr args)
    (lambda (in)
      (format #t "~s~%" (ssax:xml->sxml in '()))))
  0)
