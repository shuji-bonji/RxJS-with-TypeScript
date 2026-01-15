---
description: zipWith es un operador de combinación de RxJS que empareja el Observable original con otros Observables en orden correspondiente. Es la versión Pipeable Operator de la Creation Function zip, y es ideal para situaciones donde el orden es importante, como juegos de preguntas (preguntas y respuestas), asignación de tareas (usuarios y tareas), asignación de asientos (pasajeros y números de asiento), etc. Es útil cuando se desea transformar y procesar el stream principal mientras se empareja con otros streams.
titleTemplate: ':title | RxJS'
---

# zipWith - Emparejar por Orden

El operador `zipWith` empareja el Observable original con los otros Observables especificados **en orden correspondiente** para formar un nuevo stream.
Esta es la versión Pipeable Operator de la Creation Function `zip`.

## 🔰 Sintaxis Básica y Uso

```ts
import { of, interval } from 'rxjs';
import { zipWith, map, take } from 'rxjs';

const source1$ = of('A', 'B', 'C', 'D');
const source2$ = interval(500).pipe(take(4)); // 0, 1, 2, 3

source1$
  .pipe(
    zipWith(source2$),
    map(([letter, num]) => `${letter}${num}`)
  )
  .subscribe(console.log);

// Salida:
// A0 (después de 500ms)
// B1 (después de 1000ms)
// C2 (después de 1500ms)
// D3 (después de 2000ms)
```

- Empareja **valores en orden correspondiente** de cada Observable, uno a la vez.
- **Espera hasta que todos los Observables hayan emitido valores en orden correspondiente** antes de emitir el par.
- Cuando cualquier Observable completa, todo el stream completa.

[🌐 Documentación Oficial de RxJS - `zipWith`](https://rxjs.dev/api/operators/zipWith)


## 💡 Patrones de Uso Típicos

- **Emparejamiento de preguntas y respuestas de juego de preguntas**: Emparejar preguntas secuenciales con respuestas de usuario
- **Asignación de tareas**: Emparejar listas de usuarios con listas de tareas en secuencia
- **Asignación de asientos**: Emparejar pasajeros con números de asiento en secuencia
- **Consolidar resultados de procesamiento paralelo**: Combinar resultados de múltiples llamadas API en orden


## 🧠 Ejemplo de Código Práctico (con UI)

Ejemplo de un juego de preguntas donde las preguntas y respuestas de usuario se emparejan en orden y se puntúan.

```ts
import { fromEvent, of, from } from 'rxjs';
import { zipWith, map, take, scan } from 'rxjs';

// Construir la UI
const container = document.createElement('div');
container.innerHTML = `
  <h3>Ejemplo Práctico de zipWith: Juego de Preguntas</h3>
  <div id="question" style="font-size: 18px; margin: 10px 0;">Cargando preguntas...</div>
  <div>
    <button id="answer-a">A</button>
    <button id="answer-b">B</button>
    <button id="answer-c">C</button>
  </div>
  <div id="result" style="margin-top: 10px;"></div>
  <div id="score" style="margin-top: 10px; font-weight: bold;"></div>
`;
document.body.appendChild(container);

const questionDiv = document.getElementById('question')!;
const resultDiv = document.getElementById('result')!;
const scoreDiv = document.getElementById('score')!;

const buttonA = document.getElementById('answer-a') as HTMLButtonElement;
const buttonB = document.getElementById('answer-b') as HTMLButtonElement;
const buttonC = document.getElementById('answer-c') as HTMLButtonElement;

// Lista de preguntas (con respuestas correctas)
interface Question {
  id: number;
  text: string;
  correct: string;
}

const questions: Question[] = [
  { id: 1, text: '¿Qué significa "Rx" en RxJS?', correct: 'A' },
  { id: 2, text: '¿Qué representa Observable?', correct: 'B' },
  { id: 3, text: '¿Qué hace subscribe?', correct: 'C' }
];

// Stream de preguntas
const questions$ = of(...questions);

let currentQuestionIndex = 0;
questions$.subscribe(q => {
  if (currentQuestionIndex === 0) {
    questionDiv.textContent = `P${q.id}: ${q.text}`;
  }
});

// Stream de respuestas de usuario (clics de botón)
const getAnswer = () => new Promise<string>((resolve) => {
  const handleClick = (answer: string) => {
    resolve(answer);
    buttonA.removeEventListener('click', handleA);
    buttonB.removeEventListener('click', handleB);
    buttonC.removeEventListener('click', handleC);
  };
  const handleA = () => handleClick('A');
  const handleB = () => handleClick('B');
  const handleC = () => handleClick('C');
  buttonA.addEventListener('click', handleA);
  buttonB.addEventListener('click', handleB);
  buttonC.addEventListener('click', handleC);
});

const answers$ = from(
  Promise.all(questions.map(() => getAnswer()))
);

// Emparejar y calificar preguntas con respuestas
questions$
  .pipe(
    zipWith(answers$),
    map(([question, answer]) => ({
      question: question.text,
      answer,
      correct: question.correct,
      isCorrect: answer === question.correct
    })),
    scan((acc, result) => ({
      ...result,
      totalScore: acc.totalScore + (result.isCorrect ? 1 : 0)
    }), { totalScore: 0 } as any)
  )
  .subscribe((result) => {
    const status = result.isCorrect ? '✅ Correcto' : '❌ Incorrecto';
    resultDiv.innerHTML += `<div>${status}: ${result.question} - Tu respuesta: ${result.answer}</div>`;
    scoreDiv.textContent = `Puntuación actual: ${result.totalScore} / ${currentQuestionIndex + 1}`;
    currentQuestionIndex++;

    // Mostrar siguiente pregunta
    if (currentQuestionIndex < questions.length) {
      questionDiv.textContent = `P${questions[currentQuestionIndex].id}: ${questions[currentQuestionIndex].text}`;
    } else {
      questionDiv.textContent = '¡Todas las preguntas completadas!';
      buttonA.disabled = true;
      buttonB.disabled = true;
      buttonC.disabled = true;
    }
  });
```

- Cada vez que un usuario responde, se **empareja con la pregunta correspondiente** y se puntúa.
- El orden está garantizado, por lo que se mantiene la correspondencia: **Respuesta 1 para Pregunta 1, Respuesta 2 para Pregunta 2**, y así sucesivamente.


## 🔄 Diferencia con la Creation Function `zip`

### Diferencias Básicas

| | `zip` (Creation Function) | `zipWith` (Pipeable Operator) |
|:---|:---|:---|
| **Ubicación de Uso** | Usado como función independiente | Usado dentro de cadena `.pipe()` |
| **Sintaxis** | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **Primer Stream** | Trata todos por igual | Trata como stream principal |
| **Ventaja** | Simple y legible | Fácil de combinar con otros operadores |

### Ejemplos de Uso Específicos

**Creation Function Recomendada Solo para Emparejamiento Simple**

```ts
import { zip, of } from 'rxjs';

const names$ = of('Alice', 'Bob', 'Charlie');
const ages$ = of(25, 30, 35);
const cities$ = of('Tokio', 'Osaka', 'Kioto');

// Simple y legible
zip(names$, ages$, cities$).subscribe(([name, age, city]) => {
  console.log(`${name} (${age} años) - ${city}`);
});
// Salida:
// Alice (25 años) - Tokio
// Bob (30 años) - Osaka
// Charlie (35 años) - Kioto
```

**Pipeable Operator Recomendado Cuando Se Agrega Procesamiento de Transformación al Stream Principal**

```ts
import { of } from 'rxjs';
import { zipWith, map, filter } from 'rxjs';

const users$ = of(
  { id: 1, name: 'Alice', active: true },
  { id: 2, name: 'Bob', active: false },
  { id: 3, name: 'Charlie', active: true }
);

const tasks$ = of('Tarea A', 'Tarea B', 'Tarea C');

// ✅ Versión Pipeable Operator - completada en un pipeline
users$
  .pipe(
    filter(user => user.active),    // Solo usuarios activos
    map(user => user.name),         // Extraer solo nombre
    zipWith(tasks$)                 // Emparejar con tareas
  )
  .subscribe(([user, task]) => {
    console.log(`Asignar ${task} a ${user}`);
  });
// Salida:
// Asignar Tarea A a Alice
// Asignar Tarea B a Charlie

// ❌ Versión Creation Function - se vuelve verbosa
import { zip } from 'rxjs';
zip(
  users$.pipe(
    filter(user => user.active),
    map(user => user.name)
  ),
  tasks$
).subscribe(([user, task]) => {
  console.log(`Asignar ${task} a ${user}`);
});
```

### Resumen

- **`zip`**: Óptimo para simplemente emparejar múltiples streams
- **`zipWith`**: Óptimo cuando se desea transformar/procesar el stream principal mientras se empareja con otros streams


## ⚠️ Notas Importantes

### Tiempo de Completación

Cuando cualquier Observable completa, todo el stream completa.

```ts
import { of, interval } from 'rxjs';
import { zipWith, take } from 'rxjs';

of(1, 2, 3).pipe(
  zipWith(
    interval(1000).pipe(take(2)),  // Emite solo 2 valores
  )
).subscribe({
  next: console.log,
  complete: () => console.log('✅ Completo')
});
// Salida: [1, 0] → [2, 1] → ✅ Completo
// * interval$ emitió solo 2 valores y completó, por lo que 3 no se empareja
```

### Sincronización de Tiempo de Emisión

`zipWith` espera **hasta que todos los Observables hayan emitido valores en orden correspondiente**.

```ts
import { interval } from 'rxjs';
import { zipWith, take, map } from 'rxjs';

const fast$ = interval(100).pipe(take(5), map(i => `Rápido: ${i}`));
const slow$ = interval(1000).pipe(take(5), map(i => `Lento: ${i}`));

fast$
  .pipe(zipWith(slow$))
  .subscribe(console.log);
// Salida (cada 1 segundo):
// ['Rápido: 0', 'Lento: 0']
// ['Rápido: 1', 'Lento: 1']
// ['Rápido: 2', 'Lento: 2']
// ['Rápido: 3', 'Lento: 3']
// ['Rápido: 4', 'Lento: 4']
// * fast$ es rápido, pero espera a que slow$ emita, por lo que los pares se emiten cada segundo
```

### Diferencia con combineLatestWith

`combineLatestWith` **siempre combina** los últimos valores, mientras que `zipWith` **empareja basándose en el orden**.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(100).pipe(take(3)); // 0, 1, 2
const source2$ = interval(200).pipe(take(2)); // 0, 1

// zipWith: Emparejar por orden
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// Salida: [0, 0] → [1, 1] → Completo
// * Dado que source2$ completó, el 2 de source1$ no se empareja

// combineLatestWith: Combinar últimos valores
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// Salida: [0, 0] → [1, 0] → [1, 1] → [2, 1]
// * Emite la última combinación de valores cada vez que cualquiera emite
```

### Manejo de Errores

Si ocurre un error en cualquier Observable, todo el stream termina con un error.

```ts
import { throwError, of } from 'rxjs';
import { zipWith, catchError } from 'rxjs';

of(1, 2, 3).pipe(
  zipWith(
    throwError(() => new Error('Ocurrió un error')).pipe(
      catchError(err => of('Error recuperado'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Salida: [1, 'Error recuperado']
```


## 📚 Operadores Relacionados

- **[zip](/es/guide/creation-functions/combination/zip)** - Versión Creation Function
- **[combineLatestWith](/es/guide/operators/combination/combineLatestWith)** - Siempre combinar últimos valores
- **[withLatestFrom](/es/guide/operators/combination/withLatestFrom)** - Combinar solo cuando el stream principal emite
