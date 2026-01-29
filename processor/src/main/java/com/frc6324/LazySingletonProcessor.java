package com.frc6324;

import com.frc6324.lib.LazySingleton;
import com.google.auto.service.AutoService;
import java.util.Set;
import javax.annotation.processing.*;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.*;
import javax.tools.Diagnostic;

@SupportedAnnotationTypes("com.frc6324.lib.LazySingleton")
@SupportedSourceVersion(SourceVersion.RELEASE_17)
@AutoService(Processor.class)
public class LazySingletonProcessor extends AbstractProcessor {
  @Override
  public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
    for (Element element : roundEnv.getElementsAnnotatedWith(LazySingleton.class)) {
      if (element.getKind() != ElementKind.CLASS) {
        error(element, "@LazySingleton can only be applied to classes.");
        continue;
      }
    }

    return true;
  }

  private void error(Element element, String message) {
    processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR, message, element);
  }
}
