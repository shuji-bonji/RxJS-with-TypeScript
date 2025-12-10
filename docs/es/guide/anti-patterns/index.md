---
description: Una guía práctica para comprender los anti-patrones de RxJS y escribir código más robusto y mantenible. Explicaremos 16 errores comunes y cómo tratarlos, incluyendo el mal uso de Subject, subscribe anidados, ramificación condicional en subscribe, fugas de memoria y mal uso de shareReplay.
---

# Colección de Anti-Patrones de RxJS

RxJS es una poderosa biblioteca de programación reactiva, pero cuando se usa incorrectamente, puede ser un caldo de cultivo para errores y mantenibilidad reducida. En esta sección, presentaremos algunos errores comunes al usar RxJS con TypeScript y las mejores prácticas para evitarlos.

## Propósito de esta sección

- **Prevenir errores**: evitar problemas de implementación comprendiendo los errores comunes antes de que ocurran
- **Mejorar la Mantenibilidad**: aprender patrones de código que sean fáciles de leer y probar
- **Optimizar el rendimiento**: aprender técnicas para evitar fugas de memoria y procesamiento innecesario

## Lista de anti-patrones

Esta sección cubre los siguientes 17 anti-patrones.

### 🔴 Problemas Críticos

Estos patrones pueden tener un impacto grave en su aplicación.

| Patrón | Problema | Impacto |
|---|---|---|
| **[Publicación externa de Subject](./common-mistakes#1-publicacion-externa-de-subject)** | Publicar el Subject tal cual, y hacer posible llamar `next()` desde el exterior | Imprevisibilidad de la gestión del estado, difícil de depurar |
| **[Subscribe anidado](./common-mistakes#2-subscribe-anidado-callback-hell)** | Llamar más `subscribe` en `subscribe` | Callback hell, complicación del manejo de errores |
| **[Proliferación desenfrenada de flags de gestión de estado](./flag-management)** | Gestión del estado con 17 flags booleanos, el pensamiento imperativo aún permanece | Baja legibilidad, difícil de mantener, semillero de errores |
| **[Anidamiento de sentencias if en subscribe](./subscribe-if-hell)** | Ramificación condicional compleja (anidamiento de 3 o más niveles) en `subscribe` | Baja legibilidad, difícil de probar, viola la filosofía declarativa |
| **[Olvido de unsubscribe](./common-mistakes#3-olvido-de-unsubscribe-fuga-de-memoria)** | No cancelar la suscripción a streams infinitos | Fuga de memoria, desperdicio de recursos |
| **[Mal uso de shareReplay](./common-mistakes#4-mal-uso-de-sharereplay)** | Usar sin comprender cómo funciona `shareReplay` | Referencias a datos antiguos, fugas de memoria |

### 🟡 Problemas que Requieren Atención

Estos pueden ser problemas en ciertas situaciones.

| Patrón | Problema | Impacto |
|---|---|---|
| **[Efectos secundarios en map](./common-mistakes#5-efectos-secundarios-en-map)** | Cambiar el estado en el operador `map` | Comportamiento impredecible, difícil de probar |
| **[Ignorar Cold/Hot](./common-mistakes#6-ignorar-diferencias-entre-observable-cold-hot)** | No considerar la naturaleza del Observable | Ejecución duplicada, comportamiento inesperado |
| **[Mezcla con Promise](./promise-observable-mixing)** | No convertir Promise y Observable correctamente | No cancelable, mal manejo de errores |
| **[Ignorar backpressure](./common-mistakes#8-ignorar-backpressure)** | No controlar eventos de alta frecuencia | Degradación del rendimiento, congelamiento de la UI |

### 🔵 Problemas de Calidad del Código

Estos no son errores directos, pero son factores que reducen la calidad del código.

| Patrón | Problema | Impacto |
|---|---|---|
| **[Supresión de errores](./common-mistakes#9-supresion-de-errores)** | No manejar los errores correctamente | Dificultades de depuración, mala experiencia del usuario |
| **[Fugas de eventos DOM](./common-mistakes#10-fugas-de-suscripcion-de-eventos-dom)** | No liberar los event listeners del DOM | Fugas de memoria, degradación del rendimiento |
| **[Falta de seguridad de tipos](./common-mistakes#11-falta-de-seguridad-de-tipos-uso-excesivo-de-any)** | Uso excesivo de `any` | Errores en tiempo de ejecución, dificultades de refactorización |
| **[Selección impropia de operadores](./common-mistakes#12-seleccion-impropia-de-operadores)** | Uso de operadores que no sirven al propósito | Ineficiencia, comportamiento inesperado |
| **[Sobrecomplicación](./common-mistakes#13-sobrecomplicacion)** | Complicar un proceso que puede escribirse de forma simple | Baja legibilidad, difícil de mantener |
| **[Infierno de one-liner](./one-liner-hell)** | Definiciones de streams, conversiones y suscripciones mezcladas | Difícil de depurar, difícil de probar, baja legibilidad |
| **[Cambios de estado en subscribe](./common-mistakes#14-cambios-de-estado-en-subscribe)** | Cambio de estado directamente en `subscribe` | Difícil de probar, causa errores |
| **[Falta de pruebas](./common-mistakes#15-falta-de-pruebas)** | No hay pruebas escritas para el código RxJS | Regresión, dificultades de refactorización |

## Cómo proceder con el estudio

1. Aprender 15 anti-patrones en detalle en **[Errores Comunes y Cómo Corregirlos](./common-mistakes)**
2. Cada anti-patrón tiene un "ejemplo malo" y un "ejemplo bueno" de código
3. Revisar su código con **[Lista de Verificación para Evitar Anti-patrones](./checklist)**
4. Practicar las mejores prácticas y compartirlas con su equipo

## Secciones Relacionadas

Después de aprender sobre anti-patrones, consulte también las siguientes secciones.

- **[Manejo de Errores](/es/guide/error-handling/strategies)** - Estrategias apropiadas de manejo de errores
- **[Técnicas de Prueba](/es/guide/testing/unit-tests)** - Cómo probar el código RxJS
- **[Comprensión de Operadores](/es/guide/operators/)** - Cómo elegir los operadores adecuados

## Próximos Pasos

1. Comience con **[Errores Comunes y Cómo Corregirlos](./common-mistakes)** para aprender anti-patrones prácticos y sus soluciones.
2. Después de aprender, revise su código real con la **[Lista de Verificación para Evitar Anti-patrones](./checklist)**.

---

**IMPORTANTE**: Estos anti-patrones se encuentran frecuentemente en proyectos reales. Comprenderlos tempranamente le ayudará a escribir código RxJS de calidad.
