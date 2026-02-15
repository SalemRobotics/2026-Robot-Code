package com.frc6324;

import com.frc6324.lib.LazySingleton;
import com.google.auto.service.AutoService;
import java.util.Set;
import javax.annotation.processing.*;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.*;
import javax.tools.Diagnostic;

@SupportedAnnotationTypes("com.frc6324.lib.LazySingleton")
@SupportedSourceVersion(SourceVersion.RELEASE_17) // adjust as needed
@AutoService(Processor.class)
public class LazySingletonProcessor extends AbstractProcessor {
  @Override
  public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
    for (Element element : roundEnv.getElementsAnnotatedWith(LazySingleton.class)) {
      if (element.getKind() != ElementKind.CLASS) {
        error(element, "@LazySingleton can only be applied to classes.");
        continue;
      }

      TypeElement clazz = (TypeElement) element;

      if (!clazz.getModifiers().contains(Modifier.FINAL)) {
        error(clazz, "Singleton class must be final.");
      }

      boolean hasConstructor = false;
      boolean hasStaticInstance = false;
      boolean hasGetInstance = false;

      for (Element e : clazz.getEnclosedElements()) {
        switch (e.getKind()) {
          case CONSTRUCTOR -> {
            hasConstructor = true;

            if (!e.getModifiers().contains(Modifier.PRIVATE)) {
              error(e, "Constructor of singleton class must be private.");
            }
          }
          case FIELD -> {
            VariableElement field = (VariableElement) e;

            if (field.getSimpleName().contentEquals("instance")) {
              Set<Modifier> mods = field.getModifiers();

              if (!mods.contains(Modifier.PRIVATE)) {
                error(field, "Instance field must be private.");
              }

              if (!mods.contains(Modifier.STATIC)) {
                error(field, "Instance field must be static.");
              }

              if (mods.contains(Modifier.FINAL)) {
                error(field, "Lazy static instance must not be final.");
              }

              if (!processingEnv.getTypeUtils().isSameType(field.asType(), clazz.asType())) {
                error(field, "Instance must be an instance of " + clazz.getSimpleName());
              }

              hasStaticInstance = true;
            }
          }
          case METHOD -> {
            ExecutableElement method = (ExecutableElement) e;

            if (method.getSimpleName().contentEquals("getInstance")) {
              hasGetInstance = true;

              if (!method.getModifiers().contains(Modifier.STATIC)) {
                error(method, "Instance getter must be static.");
              }

              if (!processingEnv
                  .getTypeUtils()
                  .isSameType(method.getReturnType(), clazz.asType())) {
                error(
                    method, "Instance getter must return an instance of " + clazz.getSimpleName());
              }
            }
          }
          default -> {}
        }
      }

      if (!hasConstructor) {
        error(clazz, "Singleton class must have a private constructor declared.");
      }

      if (!hasStaticInstance) {
        error(clazz, "Singleton class must declare a `private static` instance field.");
      }

      if (!hasGetInstance) {
        error(
            clazz,
            "Singleton class must provide a `getInstance` method to get the singleton instance.");
      }
    }

    return true;
  }

  private void error(Element element, String message) {
    processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR, message, element);
  }
}
