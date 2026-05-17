---
description: "El operador ignoreElements es un operador de filtrado de RxJS que ignora todos los valores y sólo pasa por las finalizaciones y los errores. Esto es útil cuando se espera a que el proceso se complete."
---

# ignoreElements - sólo pasan los completados/errores

El operador `ignoreElements` **ignora todos los valores** emitidos por el Observable fuente y sólo se pasan **notificaciones de finalización y error**.

## 🔰 Sintaxis básica y uso

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valor:', value), // No solicitado
  complete: () => console.log('Completado')
});
// Salida: Completado
```

**Flujo de operación**:.
1. todos los 1, 2, 3, 4 y 5 son ignorados
2. sólo se transmiten las notificaciones de finalización

[🌐 Documentación oficial de RxJS - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Patrón de utilización típico.

- **Esperar a la finalización del proceso**: cuando no necesitas el valor y sólo quieres conocer la finalización.
- **Ejecutar sólo efectos secundarios**: ejecutar efectos secundarios con tap e ignorar valores.
- **Manejo de errores**: cuando sólo se quieren capturar errores
- **Sincronizar secuencias**: esperar a que se completen varios procesos

## 🧠 Ejemplo práctico de código 1: Esperar a la finalización del proceso de inicialización

Este es un ejemplo de espera a la finalización de múltiples procesos de inicialización.

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs';

// UICreado
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Inicialización de la aplicación';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// Función para añadir registro de estado
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// Proceso de inicialización1: Conexión a la base de datos
const initDatabase$ = from(['DBConexión a...', 'Comprobación de la tabla...', 'DBListo']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // Valores ignorados, sólo se notifica la finalización
);

// Proceso de inicialización2: Se está leyendo el archivo de configuración
const loadConfig$ = from(['Se está leyendo el archivo de configuración...', 'Análisis de configuración en curso...', 'Aplicación de configuración completada']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// Proceso de inicialización3: Autenticación de usuario
const authenticate$ = from(['Se está verificando la información de autenticación...', 'Verificación de token en curso...', 'Autenticación completada']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// Se ejecutan todos los procesos de inicialización.
addLog('Inicialización iniciada...', 'orange');

forkJoin([
  initDatabase$,
  loadConfig$,
  authenticate$
]).subscribe({
  complete: () => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#e8f5e9';
    completeMessage.style.color = 'green';
    completeMessage.style.fontWeight = 'bold';
    completeMessage.textContent = '✅ Se ha completado toda la inicialización.！La aplicación puede iniciarse.';
    addLog('Aplicación iniciada', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ Error de inicialización: ${err.message}`;
  }
});
```

- Se muestra un registro detallado de cada proceso de inicialización, pero se ignoran los valores.
- Cuando todos los procesos se han completado, se muestra un mensaje de finalización.

## 🎯 Ejemplo práctico de código 2: Esperando a que finalice la carga de archivos

Este es un ejemplo de visualización del progreso de subida de varios ficheros, pero sólo notificando la finalización.

```ts
import { from, of, concat } from 'rxjs';
import { ignoreElements, tap, delay, mergeMap } from 'rxjs';

// UICreado
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Carga de archivos';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Carga iniciada';
container.appendChild(button);

const progressArea = document.createElement('div');
progressArea.style.marginTop = '10px';
container.appendChild(progressArea);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.display = 'none';
container.appendChild(result);

interface FileUpload {
  name: string;
  size: number;
}

const files: FileUpload[] = [
  { name: 'document.pdf', size: 2500 },
  { name: 'image.jpg', size: 1800 },
  { name: 'video.mp4', size: 5000 }
];

// Proceso de carga de archivos (con indicación de progreso)
function uploadFile(file: FileUpload) {
  const fileDiv = document.createElement('div');
  fileDiv.style.marginTop = '5px';
  fileDiv.style.padding = '5px';
  fileDiv.style.border = '1px solid #ccc';
  progressArea.appendChild(fileDiv);

  const progressSteps = [0, 25, 50, 75, 100];

  return from(progressSteps).pipe(
    delay(200),
    tap(progress => {
      fileDiv.textContent = `📄 ${file.name} (${file.size}KB) - ${progress}%`;
      if (progress === 100) {
        fileDiv.style.backgroundColor = '#e8f5e9';
      }
    }),
    ignoreElements() // Valores de progreso ignorados, sólo se notifica la finalización
  );
}

button.addEventListener('click', () => {
  button.disabled = true;
  progressArea.innerHTML = '';
  result.style.display = 'none';

  // Todos los archivos se cargan secuencialmente
  from(files).pipe(
    mergeMap(file => uploadFile(file), 2) // Máx.23 archivos en paralelo
  ).subscribe({
    complete: () => {
      result.style.display = 'block';
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
      result.innerHTML = `
        <strong>✅ Carga completada</strong><br>
        ${files.length}Se ha cargado un archivo
      `;
      button.disabled = false;
    },
    error: err => {
      result.style.display = 'block';
      result.style.backgroundColor = '#ffebee';
      result.style.color = 'red';
      result.textContent = `❌ Error: ${err.message}`;
      button.disabled = false;
    }
  });
});
```

- Se muestra el progreso de cada archivo, pero los valores de progreso en sí no fluyen hacia abajo.
- Se muestra un mensaje de finalización cuando se han completado todas las cargas.

## 🆚 Comparación con operadores similares

### ignoreElements vs filter(() => false) vs take(0)

```ts
import { of } from 'rxjs';
import { ignoreElements, filter, take } from 'rxjs';

const source$ = of(1, 2, 3);

// ignoreElements: Ignorar todos los valores, la finalización se pasa a través de
source$.pipe(
  ignoreElements()
).subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('ignoreElements: Completado')
});
// Salida: ignoreElements: Completado

// filter(() => false): Filtra todos los valores, deja pasar la finalización
source$.pipe(
  filter(() => false)
).subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('filter: Completado')
});
// Salida: filter: Completado

// take(0): Completado inmediatamente
source$.pipe(
  take(0)
).subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('take(0): Completado')
});
// Salida: take(0): Completado
```

| Operador | Tratamiento de valores | Notificación de finalización | Caso de uso. |
|---|---|---|---|
| `ignorarElementos()` | Ignora todos los | pasar | **Necesario sólo para completar** (recomendado) |
| Filtro(() => false) | Filtra todos los elementos | dejar pasar | Filtrado condicional (excluir todo por casualidad). |
| Tomar(0)` | Completar inmediatamente | dejar pasar | Completar inmediatamente |

**Recomendado**: utilice `ignoreElements()` si desea ignorar intencionadamente todos los valores. La intención del código será clara.

## 🔄 Manejo de notificaciones de error.

`ignoreElements` ignora los valores, pero **pasa las notificaciones de error**.

```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('Se produce un error'))
).pipe(
  ignoreElements()
);

// Caso de éxito
success$.subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('✅ Completado'),
  error: err => console.error('❌ Error:', err.message)
});
// Salida: ✅ Completado

// Caso de error
error$.subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('✅ Completado'),
  error: err => console.error('❌ Error:', err.message)
});
// Salida: ❌ Error: Se produce un error
```

## ⚠️ Notas.

### 1. se realizan efectos secundarios

`ignoreElements` ignora los valores, pero se realizan los efectos secundarios (como `tap`).

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs';

of(1, 2, 3).pipe(
  tap(v => console.log('Efectos secundarios:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('Completado')
});
// Salida:
// Efectos secundarios: 1
// Efectos secundarios: 2
// Efectos secundarios: 3
// Completado
```

### 2. Uso con InfiniteObservable

Cuando se utiliza con InfiniteObservable, la suscripción dura para siempre ya que la finalización nunca llega.

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs';

// ❌ Caso negativo: No se ha completado
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Completado') // No solicitado
});

// ✅ Buen ejemplo: take Completado en
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('Completado') // 5Llamada después de un segundo
});
```

### 3. Tipos en TypeScript

El valor de retorno de `ignoreElements` es del tipo `Observable<never>`.

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const numbers$: Observable<number> = of(1, 2, 3);

// ignoreElements El resultado de Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value es de tipo never por lo que este bloque no se ejecuta
    console.log(value);
  },
  complete: () => console.log('Sólo se completa')
});
```

### 4. si la finalización no está garantizada

Si la fuente no se completa, el `ignoreElements` tampoco se completará.

```ts
import { NEVER } from 'rxjs';
import { ignoreElements } from 'rxjs';

// ❌ NEVERno completará ni emitirá un error
NEVER.pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Completado') // No solicitado
});
```

## 💡 Patrones de combinación prácticos

### Patrón 1: Secuencia de inicialización.

```ts
import { of, concat } from 'rxjs';
import { tap, ignoreElements, delay } from 'rxjs';

const initStep1$ = of('Step 1').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

const initStep2$ = of('Step 2').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

const initStep3$ = of('Step 3').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

// Todos los pasos se ejecutan secuencialmente
concat(initStep1$, initStep2$, initStep3$).subscribe({
  complete: () => console.log('✅ Se ha completado toda la inicialización')
});
```

### Patrón 2: Proceso de limpieza

```ts
import { from, of } from 'rxjs';
import { tap, ignoreElements, mergeMap } from 'rxjs';

interface Resource {
  id: number;
  name: string;
}

const resources: Resource[] = [
  { id: 1, name: 'Database' },
  { id: 2, name: 'Cache' },
  { id: 3, name: 'Logger' }
];

from(resources).pipe(
  mergeMap(resource =>
    of(resource).pipe(
      tap(() => console.log(`🧹 ${resource.name} Limpieza en curso...`)),
      ignoreElements()
    )
  )
).subscribe({
  complete: () => console.log('✅ Se han limpiado todos los recursos')
});
```

## 📚 Operadores relacionados.

- **[filtro](. /filtro)** - filtra valores basándose en condiciones.
- **[take](. /take)** - sólo se toman los N primeros valores.
- **[skip](. /skip)** - omite los N primeros valores.
- **[tap](. /utility/tap)** - realiza una acción lateral

## Resumen.

El operador `ignoreElements` ignora todos los valores y sólo pasa por las terminaciones y los errores.

- ✅ Ideal cuando sólo se requiere notificación de finalización.
- ✅ Se ejecutan los efectos secundarios (TAP).
- ✅ También se pasan las notificaciones de error
- ✅ Intención más clara que `filter(() => false)`.
- ⚠️ Observable infinito no se completa
- ⚠️ El tipo de valor de retorno es `Observable<nunca>`.
- ⚠️ El valor se ignora completamente, pero se realizan efectos secundarios
