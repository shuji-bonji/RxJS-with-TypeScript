---
description: dematerialize es un operador de utilidad de RxJS que restaura objetos Notification a notificaciones normales (next, error, complete) y realiza la transformación inversa de materialize. Es ideal para restaurar notificaciones después del procesamiento, filtrar o convertir errores, reordenar o almacenar en búfer notificaciones, o cualquier otra situación donde desee procesar notificaciones como datos y luego devolverlas a su formato original.
---

# dematerialize - Restaurar Objeto Notification

El operador `dematerialize` **convierte** un objeto Notification en una notificación normal (next, error, complete). Realiza la transformación inversa de `materialize`, restaurando la notificación datificada a su forma original.

## 🔰 Sintaxis Básica y Operación

Convierte un stream de objetos Notification de vuelta a un stream normal.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),     // Convertir a objeto Notification
    dematerialize()    // Restaurar
  )
  .subscribe({
    next: v => console.log('Valor:', v),
    complete: () => console.log('Completado')
  });
// Salida:
// Valor: 1
// Valor: 2
// Valor: 3
// Completado
```

[🌐 Documentación Oficial de RxJS - dematerialize](https://rxjs.dev/api/index/function/dematerialize)

## 💡 Ejemplos de Uso Típicos

- **Restaurar notificaciones después del procesamiento**: Restaurarlas a su formato original después del procesamiento con materialize
- **Filtrado de errores**: Excluir solo ciertos errores
- **Reorganizar el orden de notificaciones**: Restaurar después de ordenar notificaciones como datos
- **Restaurar después de depuración**: Restaurar operación normal después de logging, etc.

## 🧪 Ejemplo de Código Práctico 1: Filtrado Selectivo de Errores

Este es un ejemplo de excluir solo ciertos errores y procesar el resto normalmente.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize, filter } from 'rxjs';

// Creación de UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'dematerialize - Filtrado de errores';
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

// Stream con errores
const source$ = concat(
  of(1, 2),
  throwError(() => new Error('Error ignorable')),
  of(3, 4),
  throwError(() => new Error('Error crítico')),
  of(5)
);

source$
  .pipe(
    materialize(),
    filter(notification => {
      // Filtrar solo "Error ignorable"
      if (notification.kind === 'E') {
        const errorMessage = notification.error?.message || '';
        if (errorMessage.includes('ignorable')) {
          addLog(`🔇 Ignorado: ${errorMessage}`, '#fff9c4');
          return false;  // Excluir este error
        }
      }
      return true;
    }),
    dematerialize()  // Restaurar a formato original
  )
  .subscribe({
    next: v => addLog(`✅ Valor: ${v}`, '#c8e6c9'),
    error: err => addLog(`❌ Error: ${err.message}`, '#ffcdd2'),
    complete: () => addLog('Completado', '#e3f2fd')
  });
```

- Los "Errores ignorables" se excluyen y el stream continúa
- Los "Errores críticos" se pasan al manejador de errores normalmente
- Manejo selectivo de errores posible

## 🧪 Ejemplo de Código Práctico 2: Notificación Retrasada

Este es un ejemplo de almacenar temporalmente en búfer una notificación y luego restaurarla.

```ts
import { from, interval, take, delay } from 'rxjs';
import { materialize, dematerialize, bufferTime, concatMap } from 'rxjs';

// Creación de UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'dematerialize - Almacenamiento en búfer y retardo';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('es-ES', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Inicio - emitir valores cada segundo, procesar en lotes cada 2 segundos');

interval(1000)
  .pipe(
    take(6),
    materialize(),
    bufferTime(2000),      // Almacenar en búfer cada 2 segundos
    concatMap(notifications => {
      addLog2(`--- Procesando ${notifications.length} notificaciones del búfer ---`);
      return from(notifications).pipe(
        delay(500),        // Retrasar cada notificación por 0.5 segundos
        dematerialize()    // Restaurar a formato original
      );
    })
  )
  .subscribe({
    next: v => addLog2(`Valor: ${v}`),
    complete: () => addLog2('Completado')
  });
```

- Almacena en búfer notificaciones cada 2 segundos
- Recuperar del búfer y retrasar procesamiento
- Restaurar como stream original con `dematerialize`

## 🆚 Relación con materialize

```ts
import { of } from 'rxjs';
import { materialize, dematerialize, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),           // Convertir a Notification
    map(notification => {
      // Procesar como objeto Notification
      console.log('kind:', notification.kind);
      return notification;
    }),
    dematerialize()          // Restaurar
  )
  .subscribe(v => console.log('Valor:', v));
// Salida:
// kind: N
// Valor: 1
// kind: N
// Valor: 2
// kind: N
// Valor: 3
// kind: C
```

| Flujo de Proceso | Descripción |
|:---|:---|
| Stream original | Valor normal (next), error (error), finalización (complete) |
| ↓ `materialize()` | Stream de objeto Notification |
| Procesamiento intermedio | Procesamiento y filtrado como Notification |
| ↓ `dematerialize()` | Restaurar a stream normal |
| Stream final | Valor normal, error, complete |

## ⚠️ Notas Importantes

### 1. Las Notificaciones de Error se Convierten en Errores Reales

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Convertir cada Observable a objeto de notificación con materialize()
concat(
  of(1).pipe(materialize()),
  throwError(() => new Error('Error')).pipe(materialize()),
  of(2).pipe(materialize())  // No se ejecuta después del error
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Valor:', v),
    error: err => console.log('Error:', err.message)
  });
// Salida:
// Valor: 1
// Error: Error
```

Cuando se alcanza una notificación de error, el stream se interrumpe con un error.

### 2. La Notificación de Finalización Completa el Stream

```ts
import { of, EMPTY, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Convertir cada Observable a objeto de notificación con materialize()
concat(
  of(1).pipe(materialize()),
  of(2).pipe(materialize()),
  EMPTY.pipe(materialize()),  // Notificación de finalización
  of(3).pipe(materialize())   // No se ejecuta después de la finalización
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Valor:', v),
    complete: () => console.log('Completado')
  });
// Salida:
// Valor: 1
// Valor: 2
// Completado
```

No se emite ningún valor después de la notificación de finalización.

### 3. Objeto Notification Inválido

El `dematerialize` espera un objeto Notification correcto.

```ts
import { of } from 'rxjs';
import { dematerialize } from 'rxjs';

// ❌ Pasar valores normales a dematerialize causa error
of(1, 2, 3)
  .pipe(
    dematerialize()  // No es un objeto Notification
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Error:', err.message)
  });
// Ocurre error
```

## Ejemplos de Combinación Práctica

```ts
import { interval, throwError, of, concat } from 'rxjs';
import { materialize, dematerialize, take, mergeMap, map } from 'rxjs';

// Ejemplo de convertir errores en advertencias
interval(500)
  .pipe(
    take(10),
    mergeMap(value => {
      // Generar error solo cuando 5
      if (value === 5) {
        return throwError(() => new Error(`Error en valor ${value}`));
      }
      return of(value);
    }),
    materialize(),
    map(notification => {
      // Convertir errores en mensajes de advertencia
      if (notification.kind === 'E') {
        console.warn('Advertencia:', notification.error?.message);
        // Emitir valor especial en lugar de error (generado por materialize())
        return { kind: 'N' as const, value: -1 };
      }
      return notification;
    }),
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Valor:', v),
    error: err => console.error('Error:', err),  // No se llama
    complete: () => console.log('Completado')
  });
// Salida:
// Valor: 0, 1, 2, 3, 4
// Advertencia: Error en valor 5
// Valor: -1  (en lugar de error)
// Valor: 6, 7, 8, 9
// Completado
```

## 📚 Operadores Relacionados

- **[materialize](./materialize)** - Convertir notificación a objeto Notification
- **[catchError](/es/guide/error-handling/retry-catch)** - Manejo de errores
- **[retry](./retry)** - Reintentar en caso de error

## ✅ Resumen

El operador `dematerialize` devuelve el objeto Notification a una notificación normal.

- ✅ Conversión inversa de `materialize`
- ✅ Restaura la notificación a su formato original después del procesamiento
- ✅ Permite filtrado y conversión de errores
- ✅ Se puede usar para reordenar o almacenar en búfer notificaciones
- ⚠️ Las notificaciones de error actúan como errores reales
- ⚠️ La notificación de finalización completa el stream
- ⚠️ Requiere objeto Notification correcto
