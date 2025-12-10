---
description: El operador delayWhen controla dinámicamente el momento de retardo de cada valor con un Observable separado para lograr un procesamiento de retardo flexible según las condiciones.
---

# delayWhen - Control de Retardo Dinámico

El operador `delayWhen` determina dinámicamente el tiempo de retardo para cada valor **con un Observable individual**. Mientras que el operador `delay` proporciona un retardo de tiempo fijo, `delayWhen` puede aplicar un retardo diferente para cada valor.

## 🔰 Sintaxis Básica y Operación

Especifica una función que devuelve un Observable que determina el retardo para cada valor.

```ts
import { of, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    delayWhen(value => {
      const delayTime = value === 'B' ? 2000 : 1000;
      return timer(delayTime);
    })
  )
  .subscribe(console.log);
// Salida:
// A (después de 1 segundo)
// C (después de 1 segundo)
// B (después de 2 segundos)
```

En este ejemplo, solo el valor `'B'` tendrá un retardo de 2 segundos aplicado, los otros tendrán un retardo de 1 segundo.

[🌐 Documentación Oficial de RxJS - delayWhen](https://rxjs.dev/api/index/function/delayWhen)

## 💡 Ejemplos de Uso Típicos

- **Retardo basado en valores**: Cambiar retardo basado en prioridad o tipo
- **Retardo basado en eventos externos**: Esperar interacción del usuario o completar otros streams
- **Retardo condicional**: Retrasar solo un valor específico
- **Control de temporización asíncrona**: Esperar respuesta de API o disponibilidad de datos

## 🧪 Ejemplo de Código Práctico 1: Retardo por Prioridad

Este es un ejemplo de control del momento de procesamiento según la prioridad de la tarea.

```ts
import { from, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

// Creación de UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'delayWhen - Retardo basado en prioridad';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '300px';
output.style.overflow = 'auto';
container.appendChild(output);

interface Task {
  id: number;
  name: string;
  priority: 'high' | 'medium' | 'low';
}

const tasks: Task[] = [
  { id: 1, name: 'Tarea A', priority: 'low' },
  { id: 2, name: 'Tarea B', priority: 'high' },
  { id: 3, name: 'Tarea C', priority: 'medium' },
  { id: 4, name: 'Tarea D', priority: 'high' },
  { id: 5, name: 'Tarea E', priority: 'low' }
];

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const time = now.toLocaleTimeString('es-ES', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${time}] ${message}`;
  output.appendChild(logItem);
}

addLog('Procesamiento iniciado', '#e3f2fd');

from(tasks)
  .pipe(
    delayWhen(task => {
      // Establecer tiempo de retardo según prioridad
      let delayTime: number;
      switch (task.priority) {
        case 'high':
          delayTime = 500;  // Alta prioridad: 0.5 segundos
          break;
        case 'medium':
          delayTime = 1500; // Prioridad media: 1.5 segundos
          break;
        case 'low':
          delayTime = 3000; // Baja prioridad: 3 segundos
          break;
      }
      return timer(delayTime);
    })
  )
  .subscribe({
    next: task => {
      const colors = {
        high: '#c8e6c9',
        medium: '#fff9c4',
        low: '#ffccbc'
      };
      addLog(
        `Procesando ${task.name} (prioridad: ${task.priority})`,
        colors[task.priority]
      );
    },
    complete: () => {
      addLog('Todas las tareas completadas', '#e3f2fd');
    }
  });
```

- Las tareas de alta prioridad se procesan después de 0.5 segundos
- Las tareas de prioridad media se procesan después de 1.5 segundos, baja prioridad después de 3 segundos
- Realiza el orden de procesamiento según la importancia de la tarea

## 🧪 Ejemplo de Código Práctico 2: Retardo Debido a Eventos Externos

Este es un ejemplo de esperar un clic del usuario antes de emitir un valor.

```ts
import { of, fromEvent } from 'rxjs';
import { delayWhen, take, tap } from 'rxjs';

// Creación de UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'delayWhen - Espera de clic';
container2.appendChild(title2);

const button = document.createElement('button');
button.textContent = 'Haga clic para mostrar el siguiente valor';
button.style.marginBottom = '10px';
container2.appendChild(button);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.minHeight = '100px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

let clickCount = 0;

of('Mensaje 1', 'Mensaje 2', 'Mensaje 3')
  .pipe(
    tap(msg => {
      addLog2(`Esperando: ${msg} (por favor haga clic en el botón)`);
      button.textContent = `Haga clic para mostrar "${msg}"`;
    }),
    delayWhen(() => {
      // Retrasar hasta que ocurra el evento de clic
      return fromEvent(button, 'click').pipe(take(1));
    })
  )
  .subscribe({
    next: msg => {
      clickCount++;
      addLog2(`✅ Mostrado: ${msg}`);
      if (clickCount < 3) {
        button.disabled = false;
      } else {
        button.textContent = 'Completo';
        button.disabled = true;
      }
    },
    complete: () => {
      addLog2('--- Todos los mensajes mostrados ---');
    }
  });
```

- Cada valor se emite después de esperar un clic del usuario
- El control de retardo activado por eventos externos es posible
- Se puede aplicar al procesamiento de secuencias interactivas

## 🆚 Comparación con delay

```ts
import { of, timer } from 'rxjs';
import { delay, delayWhen } from 'rxjs';

// delay - retardo de tiempo fijo
of(1, 2, 3)
  .pipe(delay(1000))
  .subscribe(console.log);
// Todos los valores retrasados por 1 segundo

// delayWhen - diferente retardo por valor
of(1, 2, 3)
  .pipe(
    delayWhen(value => timer(value * 1000))
  )
  .subscribe(console.log);
// 1 después de 1 segundo, 2 después de 2 segundos, 3 después de 3 segundos
```

| Operador | Control de Retardo | Caso de Uso |
|:---|:---|:---|
| `delay` | Tiempo fijo | Retardo uniforme simple |
| `delayWhen` | Dinámico (por valor) | Retardo condicional, espera de evento externo |

## ⚠️ Notas Importantes

### 1. El Observable de Retardo se Genera Nuevamente Cada Vez

```ts
// ❌ Mal ejemplo: Reutilizar la misma instancia de Observable
const delayObs$ = timer(1000);
source$.pipe(
  delayWhen(() => delayObs$)  // No funcionará desde la 2da vez
).subscribe();

// ✅ Buen ejemplo: Generar nuevo Observable cada vez
source$.pipe(
  delayWhen(() => timer(1000))
).subscribe();
```

### 2. Cuando el Observable de Retardo No se Completa

```ts
import { of, NEVER } from 'rxjs';
import { delayWhen } from 'rxjs';

// ❌ Mal ejemplo: Devolver NEVER retrasa para siempre
of(1, 2, 3)
  .pipe(
    delayWhen(() => NEVER)  // Los valores no se emitirán
  )
  .subscribe(console.log);
// No hay salida
```

El Observable de retardo siempre debe emitir un valor o completarse.

### 3. Manejo de Errores

Si ocurre un error dentro del Observable de retardo, todo el stream tendrá un error.

```ts
import { of, throwError, timer, delayWhen } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delayWhen(value => {
      if (value === 2) {
        return throwError(() => new Error('Error de retardo'));
      }
      return timer(1000);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Error:', err.message)
  });
// Salida: 1
// Error: Error de retardo
```

## 📚 Operadores Relacionados

- **[delay](./delay)** - Retardo de tiempo fijo
- **[debounceTime](../filtering/debounceTime)** - Retardo después de que se detenga la entrada
- **[throttleTime](../filtering/throttleTime)** - Pasar valor cada período fijo
- **[timeout](./timeout)** - Control de tiempo de espera

## ✅ Resumen

El operador `delayWhen` controla dinámicamente el momento de retardo para cada valor.

- ✅ Se pueden aplicar diferentes retardos a cada valor
- ✅ Control de retardo por eventos externos y Observable
- ✅ Ajustar el momento de procesamiento según prioridad y tipo
- ⚠️ El Observable de retardo debe generarse nuevamente cada vez
- ⚠️ El Observable de retardo debe completarse o emitir un valor
- ⚠️ Tenga cuidado con el manejo de errores
