---
description: materialize es un operador de utilidad de RxJS que convierte las notificaciones de Observable (next, error, complete) en objetos Notification. Es ideal para situaciones donde desea manipular la notificación en sí, como manejar errores como datos, depuración y registro de notificaciones, grabación de meta-información, etc. dematerialize permite restaurar el formato original y procesamiento de notificaciones con seguridad de tipos con inferencia de tipos de TypeScript.
---

# materialize - Objetizar Notificaciones

El operador `materialize` convierte las **notificaciones de Observable (next, error, complete) en objetos Notification**. Esto permite que no solo los valores sino también los errores y las finalizaciones se manejen como datos.

## 🔰 Sintaxis Básica y Operación

Convierte un stream normal en un stream de objetos Notification.

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

of(1, 2, 3)
  .pipe(materialize())
  .subscribe(notification => {
    console.log(notification);
  });
// Salida:
// Notification { kind: 'N', value: 1, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 2, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 3, error: undefined, hasValue: true }
// Notification { kind: 'C', value: undefined, error: undefined, hasValue: false }
```

La propiedad `kind` del objeto Notification:
- `'N'`: next (valor emitido)
- `'E'`: error
- `'C'`: complete

[🌐 Documentación Oficial de RxJS - materialize](https://rxjs.dev/api/index/function/materialize)

## 💡 Ejemplos de Uso Típicos

- **Dataminería de errores**: Tratar errores como parte del stream
- **Depuración y logging**: Seguimiento detallado de notificaciones
- **Registro de meta-información**: Registrar cuándo y qué tipo de notificaciones ocurren
- **Combinar streams con errores**: Manejar errores en múltiples streams de manera unificada

## 🧪 Ejemplo de Código Práctico 1: Tratar Errores como Datos

Este ejemplo muestra cómo tratar errores que normalmente interrumpirían un stream como datos y continuar.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, map } from 'rxjs';

// Creación de UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'materialize - Dataminería de errores';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.appendChild(logItem);
}

// Manejo normal de errores (stream interrumpido)
addLog('--- Manejo normal de errores ---', '#e3f2fd');
concat(
  of(1, 2),
  throwError(() => new Error('Ocurrió un error')),
  of(3, 4)  // No se ejecuta aquí
).subscribe({
  next: v => addLog(`Valor: ${v}`, '#c8e6c9'),
  error: err => addLog(`❌ Error: ${err.message}`, '#ffcdd2'),
  complete: () => addLog('Completado', '#e3f2fd')
});

// Usando materialize (stream continúa)
setTimeout(() => {
  addLog('--- Usando materialize ---', '#e3f2fd');

  concat(
    of(1, 2),
    throwError(() => new Error('Ocurrió un error')),
    of(3, 4)
  )
    .pipe(
      materialize(),
      map(notification => {
        if (notification.kind === 'N') {
          return `Valor: ${notification.value}`;
        } else if (notification.kind === 'E') {
          return `Error (dataminado): ${notification.error?.message}`;
        } else {
          return 'Completado';
        }
      })
    )
    .subscribe({
      next: msg => {
        const color = msg.includes('Error') ? '#fff9c4' : '#c8e6c9';
        addLog(msg, color);
      },
      complete: () => addLog('Stream completado', '#e3f2fd')
    });
}, 1000);
```

- Los errores normales interrumpen el stream
- Con `materialize`, los errores se tratan como datos y el stream continúa

## 🧪 Ejemplo de Código Práctico 2: Logging de Depuración

Aquí hay un ejemplo que registra todas las notificaciones en detalle.

```ts
import { interval, throwError } from 'rxjs';
import { materialize, take, mergeMap } from 'rxjs';

// Creación de UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'materialize - Logging de depuración';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '250px';
output2.style.overflow = 'auto';
output2.style.fontFamily = 'monospace';
output2.style.fontSize = '12px';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('es-ES', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.marginBottom = '2px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

interval(500)
  .pipe(
    take(5),
    mergeMap(value => {
      // Generar error cuando el valor es 3
      if (value === 3) {
        return throwError(() => new Error('Error en valor 3'));
      }
      return of(value);
    }),
    materialize()
  )
  .subscribe({
    next: notification => {
      switch (notification.kind) {
        case 'N':
          addLog2(`[NEXT] valor: ${notification.value}`);
          break;
        case 'E':
          addLog2(`[ERROR] ${notification.error?.message}`);
          break;
        case 'C':
          addLog2('[COMPLETE]');
          break;
      }
    },
    complete: () => {
      addLog2('--- Observador completado ---');
    }
  });
```

- Registro uniforme de todos los tipos de notificación (next, error, complete)
- Rastrea el orden en que ocurren las notificaciones con marcas de tiempo
- Útil para depuración y monitoreo

## 🆚 Comparación con Streams Normales

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

// Stream normal
of(1, 2, 3).subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('Completado')
});
// Salida:
// Valor: 1
// Valor: 2
// Valor: 3
// Completado

// Usando materialize
of(1, 2, 3)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Notificación:', n),
    complete: () => console.log('Completado')
  });
// Salida:
// Notificación: Notification { kind: 'N', value: 1, ... }
// Notificación: Notification { kind: 'N', value: 2, ... }
// Notificación: Notification { kind: 'N', value: 3, ... }
// Notificación: Notification { kind: 'C', ... }
// Completado
```

## Manipular Objeto Notification

```ts
import { of } from 'rxjs';
import { materialize, map } from 'rxjs';

of(10, 20, 30)
  .pipe(
    materialize(),
    map(notification => {
      // Propiedades del objeto Notification
      return {
        kind: notification.kind,           // 'N', 'E', 'C'
        hasValue: notification.hasValue,   // Tiene valor
        value: notification.value,         // Valor (para next)
        error: notification.error          // Error (para error)
      };
    })
  )
  .subscribe(console.log);
// Salida:
// { kind: 'N', hasValue: true, value: 10, error: undefined }
// { kind: 'N', hasValue: true, value: 20, error: undefined }
// { kind: 'N', hasValue: true, value: 30, error: undefined }
// { kind: 'C', hasValue: false, value: undefined, error: undefined }
```

## ⚠️ Notas Importantes

### 1. Los Errores No Interrumpen el Stream

Al usar `materialize`, los errores se tratan como datos y el stream no se interrumpe.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize } from 'rxjs';

concat(
  of(1),
  throwError(() => new Error('Error')),
  of(2)
)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Notificación:', n.kind),
    error: () => console.log('Manejador de error'),  // No se llama
    complete: () => console.log('Completado')
  });
// Salida:
// Notificación: N
// Notificación: E  ← Los errores también se tratan como next
// Completado
```

### 2. Combinación con dematerialize

Los streams transformados con `materialize` se pueden restaurar con `dematerialize`.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),
    // Algún procesamiento aquí
    dematerialize()  // Restaurar
  )
  .subscribe(console.log);
// Salida: 1, 2, 3
```

### 3. Impacto en el Rendimiento

Hay una sobrecarga en la generación de objetos Notification. Use solo cuando sea necesario en un entorno de producción.

## 📚 Operadores Relacionados

- **[dematerialize](./dematerialize)** - Revertir objeto Notification a notificación normal
- **[tap](./tap)** - Realizar un efecto secundario (con fines de depuración)
- **[catchError](/es/guide/error-handling/retry-catch)** - Manejo de errores

## ✅ Resumen

El operador `materialize` convierte una notificación en un objeto Notification.

- ✅ Puede manejar errores como datos
- ✅ Útil para depuración y logging
- ✅ Puede registrar meta-información sobre notificaciones
- ✅ Se puede deshacer con `dematerialize`
- ⚠️ Los errores ya no interrumpirán el stream
- ⚠️ Tenga en cuenta la sobrecarga de rendimiento
