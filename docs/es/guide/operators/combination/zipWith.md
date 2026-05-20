---
description: "zipWith es un operador join de RxJS que alinea y empareja el Observable original con los valores ordenados correspondientes de otros Observables. Es ideal para uniones de flujo críticas para el orden, como la combinación de los resultados del procesamiento paralelo con garantías de orden, la asignación de ID a datos, la sincronización de datos relacionados publicados en diferentes momentos, etc. La versión del operador pipeable es conveniente para su uso dentro de pipelines."
---

# zipWith - empareja los valores correspondientes

El operador `zipWith` agrupa los **valores ordenados correspondientes** emitidos por el Observable original y los otros Observables especificados.
Espera a que los valores lleguen uno a uno desde todos los Observable y crea pares cuando están listos.
Esta es la versión Pipeable Operator del `zip` de Creation Function.

## 🔰 Sintaxis básica y uso

```ts
import { of, interval } from 'rxjs';
import { zipWith, map, take } from 'rxjs';

const letters$ = of('A', 'B', 'C', 'D');
const numbers$ = interval(1000).pipe(
  map(val => val * 10),
  take(3)
);

letters$
  .pipe(zipWith(numbers$))
  .subscribe(([letter, number]) => {
    console.log(`${letter} - ${number}`);
  });

// Salida:
// A - 0
// B - 10
// C - 20
// (Dno se emite porque no hay valor correspondiente)
```

- Se emite un par desde cada Observable **cuando uno de los valores se completa**.
- Cuando se completa un Observable, se descartan los valores restantes.

[🌐 Documentación oficial de RxJS - `zipWith`](https://rxjs.dev/api/operators/zipWith)

## 💡 Patrón de utilización típico.

- **Combinar resultados de procesamiento paralelo en orden garantizado**: emparejando los resultados de múltiples llamadas a la API.
- **Mapeo de IDs a datos**: combinación de IDs de usuario con los datos de perfil correspondientes.
- **Sincronización de flujos**: sincronización de datos relacionados emitidos en diferentes momentos.

## 🧠 Ejemplos prácticos de código (con interfaz de usuario)

Ejemplo de emparejamiento de una lista de ID de usuario y los nombres de usuario correspondientes en secuencia.

```ts
import { from, of } from 'rxjs';
import { zipWith, delay, concatMap } from 'rxjs';

// Creación de un área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>zipWith Ejemplos prácticos de:</h3>';
document.body.appendChild(output);

// UsuarioIDStream (publicado inmediatamente)
const userIds$ = from([101, 102, 103, 104]);

// Flujo de nombre de usuario (emitido cada1(emitido cada segundo)
const userNames$ = from(['Alice', 'Bob', 'Carol']).pipe(
  concatMap(name => of(name).pipe(delay(1000)))
);

// zipy se muestra
userIds$
  .pipe(zipWith(userNames$))
  .subscribe(([id, name]) => {
    const item = document.createElement('div');
    item.textContent = `👤 UsuarioID ${id}: ${name}`;
    output.appendChild(item);
  });

// Salida:
// 👤 UsuarioID 101: Alice
// 👤 UsuarioID 102: Bob
// 👤 UsuarioID 103: Carol
// (104no se muestra porque no hay nombre correspondiente)
```

- Los ID y los nombres se emparejan en una correspondencia **uno a uno**.
- Cuando se completa uno, se descartan los valores restantes.

## 🔄 Diferencia con la Creation Function `zip`.

### Diferencias básicas.

|  | `zip` (Creation Function). | `zipWith` (Pipeable Operator) |
|---|---|---|
| **Dónde se utiliza** | Usado como función independiente | Usado en la cadena `.pipe()`. |
| **Cómo se escribe | zip(obs1$, obs2$, obs3$)`. | obs1$.pipe(zipCon(obs2$, obs3$))`. |
| **Primer flujo**. | Tratar todos como iguales | Tratar como flujo principal |
| **Ventajas**. | Simple y fácil de leer | Fácil de combinar con otros operadores |

### Ejemplos concretos de uso

**Para el emparejamiento simple, se recomienda la Creation Function**.

```ts
import { zip, of } from 'rxjs';

const questions$ = of('El nombre es？', 'La edad es？', 'La dirección es？');
const answers$ = of('Taro', '30', 'Tokio');
const scores$ = of(10, 20, 30);

// Simple y legible
zip(questions$, answers$, scores$).subscribe(([q, a, s]) => {
  console.log(`Q: ${q}, A: ${a}, Puntuación: ${s}Puntuación`);
});
// Salida:
// Q: El nombre es？, A: Taro, Puntuación: 10Puntuación
// Q: La edad es？, A: 30, Puntuación: 20Puntuación
// Q: La dirección es？, A: Tokio, Puntuación: 30Puntuación
```

**Si desea añadir un proceso de transformación al flujo principal, se recomienda el Pipeable Operator**.

```ts
import { from, interval } from 'rxjs';
import { zipWith, map, take, filter } from 'rxjs';

// Lista de tareas
const tasks$ = from([
  { id: 1, name: 'Informar', priority: 'high' },
  { id: 2, name: 'Responder a correos electrónicos', priority: 'low' },
  { id: 3, name: 'Preparación de reuniones', priority: 'high' },
  { id: 4, name: 'Organizar documentos', priority: 'medium' }
]);

// Lista de responsables (1Asignados cada segundo)
const assignees$ = from(['Alice', 'Bob', 'Carol', 'Dave']).pipe(
  zipWith(interval(1000).pipe(take(4))),
  map(([name]) => name)
);

// ✅ Pipeable OperatorEdiciones - Completar en un solo paso
tasks$
  .pipe(
    filter(task => task.priority === 'high'),  // Sólo alta prioridad
    map(task => task.name),                     // Extraer nombres de tareas
    zipWith(assignees$),                        // Asignar responsable
    map(([taskName, assignee]) => ({
      task: taskName,
      assignee,
      assignedAt: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(assignment => {
    console.log(`[${assignment.assignedAt}] ${assignment.task} → Asignado a: ${assignment.assignee}`);
  });
// Salida:
// [Hora] Informar → Asignado a: Alice
// [Hora] Preparación de reuniones → Asignado a: Bob

// ❌ Creation FunctionEdiciones - Redundante
import { zip } from 'rxjs';
zip(
  tasks$.pipe(
    filter(task => task.priority === 'high'),
    map(task => task.name)
  ),
  assignees$
).pipe(
  map(([taskName, assignee]) => ({
    task: taskName,
    assignee,
    assignedAt: new Date().toLocaleTimeString()
  }))
).subscribe(assignment => {
  console.log(`[${assignment.assignedAt}] ${assignment.task} → Asignado a: ${assignment.assignee}`);
});
```

**Sincronización de datos críticos para el orden**.

```ts
import { from } from 'rxjs';
import { zipWith, map, concatMap, delay } from 'rxjs';
import { of } from 'rxjs';

// UICrear
const output = document.createElement('div');
output.innerHTML = '<h3>Juego de preguntas</h3>';
document.body.appendChild(output);

const questionArea = document.createElement('div');
questionArea.style.marginTop = '10px';
output.appendChild(questionArea);

// Lista de preguntas (preparada inmediatamente)
const questions$ = from([
  'La capital de Japón？',
  '1+1es？',
  '¿Qué planeta es la Tierra?？'
]);

// Lista de respuestas (simulando la entrada del usuario)：2(cada segundo)
const answers$ = from(['Tokio', '2', '3']).pipe(
  concatMap((answer, index) =>
    of(answer).pipe(delay((index + 1) * 2000))
  )
);

// Lista de respuestas correctas
const correctAnswers$ = from(['Tokio', '2', '3']);

// ✅ Pipeable OperatorEdiciones - Preguntas de proceso como corriente principal
questions$
  .pipe(
    zipWith(answers$, correctAnswers$),
    map(([question, answer, correct], index) => ({
      no: index + 1,
      question,
      answer,
      correct,
      isCorrect: answer === correct
    }))
  )
  .subscribe(result => {
    const div = document.createElement('div');
    div.style.marginTop = '10px';
    div.style.padding = '10px';
    div.style.border = '1px solid #ccc';
    div.style.backgroundColor = result.isCorrect ? '#e8f5e9' : '#ffebee';
    div.innerHTML = `
      <strong>Pregunta${result.no}:</strong> ${result.question}<br>
      <strong>Respuestas:</strong> ${result.answer}<br>
      <strong>Resultado:</strong> ${result.isCorrect ? '✅ Respuesta correcta！' : '❌ Respuesta incorrecta'}
    `;
    questionArea.appendChild(div);
  });
```

### Resumen.

- **`zip`**: mejor si sólo quieres mapear múltiples flujos en orden.
- **`zipWith`**: ideal si quieres unir el flujo principal con otros flujos en orden garantizado mientras transformas o procesas el flujo principal

## ⚠️ Notas.

### Para diferentes longitudes.

Cuando se completa el Observable más corto, se descartan los valores restantes del más largo.

```ts
import { of } from 'rxjs';
import { zipWith } from 'rxjs';

const short$ = of(1, 2, 3);
const long$ = of('A', 'B', 'C', 'D', 'E');

short$.pipe(zipWith(long$)).subscribe(console.log);
// Salida: [1, 'A'], [2, 'B'], [3, 'C']
// 'D'y'E'se descartan
```

### Acumulación de memoria.

Si un Observable continúa emitiendo valores, éstos se acumularán en memoria hasta que el otro se ponga al día.

```ts
import { interval} from 'rxjs';
import { zipWith, take } from 'rxjs';

// Flujos rápidos (100mspor flujo)
const fast$ = interval(100).pipe(take(10));

// Flujo de baja velocidad (1(cada segundo)
const slow$ = interval(1000).pipe(take(3));

fast$.pipe(zipWith(slow$)).subscribe(console.log);
// Salida: [0, 0] (1segundos después), [1, 1] (2segundos después), [2, 2] (3segundos después)
// fast$los valores se almacenan en memoria y se espera
```

### Diferencia con combineLatestWith.

`zipWith` empareja los valores en el orden correspondiente, mientras que `combineLatestWith` combina los últimos valores.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(1000).pipe(take(3));
const source2$ = interval(1500).pipe(take(2));

// zipWith: Se emparejan en el orden correspondiente
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// Salida: [0, 0], [1, 1]

// combineLatestWith: Combinar los últimos valores
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// Salida: [0, 0], [1, 0], [2, 0], [2, 1]
```

## 📚 Operadores relacionados.

- **[zip](/es/guide/creation-functions/combination/zip)** - Versión de Creation Function.
- **[combineLatestWith](/es/guide/operators/combination/combineLatestWith)** - Combinar último valor.
- **[withLatestFrom](/es/guide/operators/combination/withLatestFrom)** - Sólo activadores principales
