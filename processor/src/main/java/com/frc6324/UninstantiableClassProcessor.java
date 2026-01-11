package com.frc6324;

import com.frc6324.lib.UninstantiableClass;
import com.google.auto.service.AutoService;
import java.util.Set;
import javax.annotation.processing.*;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.*;
import javax.lang.model.util.ElementFilter;
import javax.tools.Diagnostic;

@SupportedAnnotationTypes("com.frc6324.lib.UninstantiableClass")
@SupportedSourceVersion(SourceVersion.RELEASE_17) // adjust as needed
@AutoService(Processor.class)
public class UninstantiableClassProcessor extends AbstractProcessor {

  @Override
  public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {

    for (Element element : roundEnv.getElementsAnnotatedWith(UninstantiableClass.class)) {

      if (element.getKind() != ElementKind.CLASS) {
        error(element, "@UninstantiableClass can only be applied to classes.");
        continue;
      }

      TypeElement clazz = (TypeElement) element;

      // Rule 1: No public or protected constructors
      for (ExecutableElement ctor : ElementFilter.constructorsIn(clazz.getEnclosedElements())) {

        Set<Modifier> mods = ctor.getModifiers();
        if (!mods.contains(Modifier.PRIVATE)) {
          error(
              ctor,
              "Classes annotated with @UninstantiableClass must have only private constructors.");
        }
      }

      // Rule 2: No instance fields
      for (VariableElement field : ElementFilter.fieldsIn(clazz.getEnclosedElements())) {

        if (!field.getModifiers().contains(Modifier.STATIC)) {
          error(field, "Instance fields are not allowed in classes marked @UninstantiableClass.");
        }
      }

      // Rule 3: No instance methods
      for (ExecutableElement method : ElementFilter.methodsIn(clazz.getEnclosedElements())) {

        if (!method.getModifiers().contains(Modifier.STATIC)) {
          error(method, "Instance methods are not allowed in classes marked @UninstantiableClass.");
        }
      }

      // Optional Rule 4: Must be final (optional!)
      if (!clazz.getModifiers().contains(Modifier.FINAL)) {
        error(clazz, "@UninstantiableClass should be final to prevent subclassing.");
      }
    }

    return true;
  }

  private void error(Element element, String message) {
    processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR, message, element);
  }
}
