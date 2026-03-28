#ifndef MAIN_CPP_HPP
#define MAIN_CPP_HPP

/* La directiva __cplusplus asegura que el compilador de C ignore el bloque extern "C"
   cuando este archivo sea incluido en main.c, evitando errores de sintaxis. */
#ifdef __cplusplus
extern "C" {
#endif

// Prototipos de las funciones principales que controlarán el ciclo de vida del programa
void Cpp_Main_Init(void);
void Cpp_Main_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_CPP_HPP */
