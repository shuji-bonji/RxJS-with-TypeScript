---
description: concatMap es un operador de transformación que procesa cada Observable en orden, esperando a que se complete el anterior antes de procesar el siguiente. Es ideal para escenarios donde el orden de ejecución es importante como ejecución en serie de llamadas API o garantía de orden en carga de archivos. La inferencia de tipos de TypeScript permite encadenar procesamiento asíncrono con seguridad de tipos, y también se explican las diferencias con mergeMap y switchMap.
---

# concatMap - Ejecutar cada Observable en orden

El operador `concatMap` convierte cada valor del stream de entrada en un Observable y **los ejecuta y combina en orden**.
La característica distintiva es que **no inicia el siguiente Observable hasta que el anterior se complete**.

## 🔰 Sintaxis básica y uso

```ts
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  concatMap(value =>
    of(`${value} completado`).pipe(delay(1000))
  )
).subscribe(console.log);

// Salida (en orden):
// A completado
// B completado
// C completado
```
- Convierte cada valor en un Observable.
- El siguiente Observable se ejecuta después de que se complete el Observable anterior.

[🌐 Documentación oficial de RxJS - concatMap](https://rxjs.dev/api/index/function/concatMap)

## 💡 Patrones típicos de uso
- Ejecución de solicitudes API donde el orden es importante
- Procesamiento de tareas basado en cola
- Control de animaciones o UI paso a paso
- Procesamiento de envío de mensajes donde el orden de envío es importante


## 🧠 Ejemplo de código práctico (con UI)

Un ejemplo donde cada vez que haces clic en el botón se genera una solicitud, y las solicitudes se procesan siempre en orden.

```ts
import { fromEvent, of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

// Crear botón
const button = document.createElement('button');
button.textContent = 'Enviar solicitud';
document.body.appendChild(button);

// Área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Evento de clic
fromEvent(button, 'click')
  .pipe(
    concatMap((_, index) => {
      const requestId = index + 1;
      console.log(`Inicio de solicitud${requestId}`);
      return of(`Respuesta${requestId}`).pipe(delay(2000));
    })
  )
  .subscribe((response) => {
    const div = document.createElement('div');
    div.textContent = `✅ ${response}`;
    output.appendChild(div);
  });

```

- Cada solicitud se envía y completa siempre en orden.
- La siguiente solicitud se emite después de que se complete la solicitud anterior.
