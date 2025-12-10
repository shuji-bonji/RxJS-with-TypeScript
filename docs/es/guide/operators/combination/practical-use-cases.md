---
description: Se explicarán casos de uso prácticos de operadores de unión de RxJS (combineLatest, forkJoin, merge, concat, withLatestFrom, etc.). Se presentarán patrones prácticos de combinación de múltiples Observables como validación de entrada de formularios e integración de API, ejecución paralela de múltiples solicitudes, sincronización de datos en tiempo real y procesamiento secuencial de streams con ejemplos de código TypeScript.
---

# Casos de Uso Prácticos

En este capítulo, presentaremos **casos de uso prácticos** que aprovechan los operadores de combinación de RxJS.
Profundice su comprensión a través de escenarios útiles para el desarrollo real de aplicaciones, como operaciones de UI y comunicación con API.

## Validación de Entrada de Formularios y Solicitudes API

Un ejemplo de validación de múltiples entradas de formulario usando `combineLatest`.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, debounceTime, startWith } from 'rxjs';

// Crear UI de formulario
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>Formulario de Registro de Usuario:</h3>';
document.body.appendChild(formContainer);

// Entrada de nombre
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Nombre: ';
formContainer.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.id = 'name';
nameInput.style.marginBottom = '10px';
nameInput.style.marginLeft = '5px';
formContainer.appendChild(nameInput);
formContainer.appendChild(document.createElement('br'));

// Entrada de email
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Email: ';
formContainer.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.id = 'email';
emailInput.style.marginBottom = '10px';
emailInput.style.marginLeft = '5px';
formContainer.appendChild(emailInput);
formContainer.appendChild(document.createElement('br'));

// Entrada de contraseña
const passwordLabel = document.createElement('label');
passwordLabel.textContent = 'Contraseña: ';
formContainer.appendChild(passwordLabel);

const passwordInput = document.createElement('input');
passwordInput.type = 'password';
passwordInput.id = 'password';
passwordInput.style.marginLeft = '5px';
formContainer.appendChild(passwordInput);
formContainer.appendChild(document.createElement('br'));

// Botón de envío
const submitButton = document.createElement('button');
submitButton.textContent = 'Registrar';
submitButton.disabled = true;
submitButton.style.marginTop = '15px';
submitButton.style.padding = '8px 16px';
formContainer.appendChild(submitButton);

// Mensaje de validación
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Validación de nombre
const name$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: value.length >= 2,
      error: value.length < 2 ? 'El nombre debe tener al menos 2 caracteres' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'El nombre debe tener al menos 2 caracteres',
  })
);

// Validación de email
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: emailRegex.test(value),
      error: !emailRegex.test(value)
        ? 'Por favor ingrese una dirección de email válida'
        : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Por favor ingrese una dirección de email válida',
  })
);

// Validación de contraseña
const password$ = fromEvent(passwordInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value;
    return {
      value,
      valid: value.length >= 6,
      error: value.length < 6 ? 'La contraseña debe tener al menos 6 caracteres' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'La contraseña debe tener al menos 6 caracteres',
  })
);

// Combinar estado de validación de todos los campos
combineLatest([name$, email$, password$])
  .pipe(debounceTime(300))
  .subscribe(([nameState, emailState, passwordState]) => {
    // Verificar si el formulario es válido
    const isFormValid =
      nameState.valid && emailState.valid && passwordState.valid;
    submitButton.disabled = !isFormValid;

    // Mostrar mensajes de error
    if (!isFormValid) {
      const errors = [
        nameState.error,
        emailState.error,
        passwordState.error,
      ].filter((error) => error !== null);

      validationMessage.textContent = errors.join('\n');
    } else {
      validationMessage.textContent = '';
    }
  });

// Evento de clic de botón de envío
fromEvent(submitButton, 'click').subscribe(() => {
  const formData = {
    name: nameInput.value,
    email: emailInput.value,
    password: passwordInput.value,
  };

  // Mostrar datos de formulario (en uso real, enviar a API)
  const successMessage = document.createElement('div');
  successMessage.textContent = '¡Registro completado!';
  successMessage.style.color = 'green';
  successMessage.style.fontWeight = 'bold';
  successMessage.style.marginTop = '10px';
  formContainer.appendChild(successMessage);

  console.log('Datos enviados:', formData);
});

```

## Solicitudes Concurrentes y Gestión de Estado de Carga

Aquí hay un ejemplo de uso de `forkJoin` para procesar múltiples solicitudes API en paralelo y resumir los resultados.

```ts
import {
  forkJoin,
  of,
  throwError,
  Observable,
  ObservableInputTuple,
} from 'rxjs';
import { catchError, delay, finalize } from 'rxjs';

// Definiciones de interfaces
interface User {
  id: number;
  name: string;
  email: string;
}

interface Post {
  id: number;
  title: string;
  content: string;
}

interface WeatherSuccess {
  city: string;
  temp: number;
  condition: string;
}

interface WeatherError {
  error: string;
}

type Weather = WeatherSuccess | WeatherError;

// Definición de tipo de resultado
interface ApiResponse {
  user: User;
  posts: Post[];
  weather: Weather;
}

// Crear elementos de UI
const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>Ejemplo de Múltiples Solicitudes API:</h3>';
document.body.appendChild(apiContainer);

const loadButton = document.createElement('button');
loadButton.textContent = 'Cargar Datos';
loadButton.style.padding = '8px 16px';
apiContainer.appendChild(loadButton);

const loadingIndicator = document.createElement('div');
loadingIndicator.style.margin = '10px 0';
loadingIndicator.style.display = 'none';
apiContainer.appendChild(loadingIndicator);

const resultContainer = document.createElement('div');
apiContainer.appendChild(resultContainer);

// Simular solicitudes API
function fetchUser(id: number): Observable<User> {
  // Solicitud exitosa
  return of({
    id,
    name: `Usuario${id}`,
    email: `usuario${id}@ejemplo.com`,
  }).pipe(
    delay(2000) // Retraso de 2 segundos
  );
}

function fetchPosts(userId: number): Observable<Post[]> {
  // Solicitud exitosa
  return of([
    { id: 1, title: `Publicación 1 de ${userId}`, content: 'Contenido...' },
    { id: 2, title: `Publicación 2 de ${userId}`, content: 'Contenido...' },
  ]).pipe(
    delay(1500) // Retraso de 1.5 segundos
  );
}

function fetchWeather(city: string): Observable<WeatherSuccess> {
  // A veces falla
  const shouldFail = Math.random() > 0.7;

  if (shouldFail) {
    return throwError(() => new Error('Error al obtener datos meteorológicos')).pipe(
      delay(1000)
    );
  }

  return of({
    city,
    temp: Math.round(15 + Math.random() * 10),
    condition: ['Soleado', 'Nublado', 'Lluvioso'][Math.floor(Math.random() * 3)],
  }).pipe(
    delay(1000) // Retraso de 1 segundo
  );
}

// Ejecutar múltiples solicitudes al hacer clic en el botón
loadButton.addEventListener('click', () => {
  // Restablecer UI
  resultContainer.innerHTML = '';
  loadingIndicator.style.display = 'block';
  loadingIndicator.textContent = 'Cargando datos...';
  loadButton.disabled = true;

  // Ejecutar múltiples solicitudes API concurrentemente
  forkJoin({
    user: fetchUser(1),
    posts: fetchPosts(1),
    weather: fetchWeather('Tokio').pipe(
      // Manejo de errores
      catchError((error: Error) => {
        console.error('Error de API meteorológica:', error);
        return of<WeatherError>({ error: error.message });
      })
    ),
  } as ObservableInputTuple<ApiResponse>)
    .pipe(
      // Limpieza al completar
      finalize(() => {
        loadingIndicator.style.display = 'none';
        loadButton.disabled = false;
      })
    )
    .subscribe((results: ApiResponse) => {
      // Mostrar información de usuario
      const userInfo = document.createElement('div');
      userInfo.innerHTML = `
      <h4>Información de Usuario</h4>
      <p>Nombre: ${results.user.name}</p>
      <p>Email: ${results.user.email}</p>
    `;
      userInfo.style.margin = '10px 0';
      userInfo.style.padding = '10px';
      userInfo.style.backgroundColor = '#f0f0f0';
      userInfo.style.borderRadius = '5px';
      resultContainer.appendChild(userInfo);

      // Mostrar publicaciones
      const postsInfo = document.createElement('div');
      postsInfo.innerHTML = `
      <h4>Publicaciones (${results.posts.length})</h4>
      <ul>
        ${results.posts
          .map((post: { title: string }) => `<li>${post.title}</li>`)
          .join('')}
      </ul>
    `;
      postsInfo.style.margin = '10px 0';
      postsInfo.style.padding = '10px';
      postsInfo.style.backgroundColor = '#f0f0f0';
      postsInfo.style.borderRadius = '5px';
      resultContainer.appendChild(postsInfo);

      // Mostrar información meteorológica
      const weatherInfo = document.createElement('div');

      if ('error' in results.weather) {
        weatherInfo.innerHTML = `
        <h4>Información Meteorológica</h4>
        <p style="color: red;">Error: ${results.weather.error}</p>
      `;
      } else {
        weatherInfo.innerHTML = `
        <h4>Información Meteorológica</h4>
        <p>Ciudad: ${results.weather.city}</p>
        <p>Temperatura: ${results.weather.temp}°C</p>
        <p>Condición: ${results.weather.condition}</p>
      `;
      }

      weatherInfo.style.margin = '10px 0';
      weatherInfo.style.padding = '10px';
      weatherInfo.style.backgroundColor = '#f0f0f0';
      weatherInfo.style.borderRadius = '5px';
      resultContainer.appendChild(weatherInfo);
    });
});

```

## Función de Búsqueda Cancelable

Aquí hay un ejemplo de combinar `withLatestFrom` y `race` para implementar una función de búsqueda con timeout o cancelable.

```ts
import { fromEvent, timer, race, of, EMPTY } from 'rxjs';
import {
  map,
  debounceTime,
  switchMap,
  tap,
  delay,
  catchError,
  takeUntil,
} from 'rxjs';

// Crear UI de búsqueda
const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Búsqueda Cancelable:</h3>';
document.body.appendChild(searchContainer);

// Campo de entrada de búsqueda
const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Ingrese término de búsqueda...';
searchInput.style.padding = '8px';
searchInput.style.width = '250px';
searchContainer.appendChild(searchInput);

// Botón de cancelar
const cancelButton = document.createElement('button');
cancelButton.textContent = 'Cancelar';
cancelButton.style.marginLeft = '10px';
cancelButton.style.padding = '8px 16px';
searchContainer.appendChild(cancelButton);

// Área de resultados de búsqueda
const resultsContainer = document.createElement('div');
resultsContainer.style.marginTop = '10px';
resultsContainer.style.minHeight = '200px';
resultsContainer.style.padding = '10px';
resultsContainer.style.border = '1px solid #ddd';
resultsContainer.style.borderRadius = '5px';
searchContainer.appendChild(resultsContainer);

// Simular solicitud de búsqueda
function searchApi(term: string) {
  console.log(`Iniciando búsqueda para "${term}"...`);

  // Simular resultados de búsqueda
  return of([
    `Resultado de búsqueda 1 para "${term}"`,
    `Resultado de búsqueda 2 para "${term}"`,
    `Resultado de búsqueda 3 para "${term}"`,
  ]).pipe(
    // Retraso aleatorio entre 2-5 segundos
    delay(2000 + Math.random() * 3000),
    // Manejo de errores
    catchError((err) => {
      console.error('Error de búsqueda:', err);
      return EMPTY;
    })
  );
}

// Evento de cancelar
const cancel$ = fromEvent(cancelButton, 'click');

// Evento de búsqueda
const search$ = fromEvent(searchInput, 'input')
  .pipe(
    // Obtener valor de entrada
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // Esperar 300ms
    debounceTime(300),
    // Ignorar búsquedas vacías
    tap((term) => {
      if (term === '') {
        resultsContainer.innerHTML = '<p>Por favor ingrese un término de búsqueda</p>';
      }
    }),
    // No procesar búsquedas vacías
    switchMap((term) => {
      if (term === '') {
        return EMPTY;
      }

      // Mostrar mensaje de búsqueda
      resultsContainer.innerHTML = '<p>Buscando...</p>';

      // Manejo de timeout (5 segundos)
      const timeout$ = timer(5000).pipe(
        tap(() => console.log('Tiempo de espera de búsqueda agotado')),
        map(() => ({ type: 'timeout', results: null }))
      );

      // Solicitud API
      const request$ = searchApi(term).pipe(
        map((results) => ({ type: 'success', results })),
        // Cancelar si se presiona el botón de cancelar
        takeUntil(
          cancel$.pipe(
            tap(() => {
              console.log('La búsqueda fue cancelada');
              resultsContainer.innerHTML = '<p>La búsqueda fue cancelada</p>';
            })
          )
        )
      );

      // Carrera entre timeout y completación de solicitud
      return race(request$, timeout$);
    })
  )
  .subscribe((response) => {
    if (response.type === 'success') {
      // Búsqueda exitosa
      resultsContainer.innerHTML = '<h4>Resultados de Búsqueda:</h4>';

      if (response.results?.length === 0) {
        resultsContainer.innerHTML += '<p>No se encontraron resultados</p>';
      } else {
        const list = document.createElement('ul');
        response.results?.forEach((result) => {
          const item = document.createElement('li');
          item.textContent = result;
          list.appendChild(item);
        });
        resultsContainer.appendChild(list);
      }
    } else if (response.type === 'timeout') {
      // Timeout
      resultsContainer.innerHTML =
        '<p style="color: red;">Tiempo de espera de búsqueda agotado. Por favor intente de nuevo.</p>';
    }
  });

```


## Comparación de Operadores de Combinación y Guía de Selección

Compare las diferencias entre múltiples operadores de combinación y ayúdele a elegir el correcto para su caso de uso.

| Operador | Tiempo | Salida | Caso de Uso |
|------------|------------|-----|------------|
| `merge` | Ejecución concurrente | Salida en orden de ocurrencia | Monitorear múltiples eventos de fuente simultáneamente |
| `concat` | Ejecución secuencial | Salida en orden | Tareas asíncronas donde el orden importa |
| `combineLatest` | Requiere al menos un valor de todas las fuentes | Combinación de todos los últimos valores | Validación de entrada de formulario |
| `zip` | Requiere valores de índice correspondiente de todas las fuentes | Combinación de valores por índice | Sincronizar datos relacionados |
| `withLatestFrom` | Cuando la fuente principal emite valor | Valor principal y último valor de otras fuentes | Combinar datos auxiliares |
| `forkJoin` | Cuando todas las fuentes completan | Último valor de cada fuente | Múltiples solicitudes API |
| `race` | Solo la primera fuente en emitir | Solo valores del stream ganador | Timeout, manejo de cancelación |

### Flujo de Decisión de Selección de Operador

1. **¿Desea recibir valores de todas las fuentes al mismo tiempo?**
   - Sí → `merge`
   - No → Siguiente

2. **¿Desea preservar el orden de las fuentes?**
   - Sí → `concat`
   - No → Siguiente

3. **¿Necesita una combinación de los últimos valores para cada fuente?**
   - Sí → ¿Cuándo combinar?
     - Para cada nuevo valor de cualquier fuente → `combineLatest`
     - Para cada valor de stream principal específico → `withLatestFrom`
   - No → Siguiente

4. **¿Necesita valores correspondientes en orden de índice?**
   - Sí → `zip`
   - No → Siguiente

5. **¿Necesita resultados después de que todas las fuentes estén completas?**
   - Sí → `forkJoin`
   - No → Siguiente

6. **¿Necesita solo el más rápido de múltiples fuentes alternativas?**
   - Sí → `race`
   - No → Reexamine el propósito


## Estrategia de Cambio

Este es un ejemplo de cambio dinámico entre múltiples fuentes de datos.

```ts
import { fromEvent, merge, interval, of } from 'rxjs';
import { map, switchMap, take, tap } from 'rxjs';

// Crear elementos de UI
const switchingContainer = document.createElement('div');
switchingContainer.innerHTML = '<h3>Cambio de Fuente de Datos:</h3>';
document.body.appendChild(switchingContainer);

// Crear botones
const source1Button = document.createElement('button');
source1Button.textContent = 'Fuente 1';
source1Button.style.margin = '5px';
source1Button.style.padding = '5px 10px';
switchingContainer.appendChild(source1Button);

const source2Button = document.createElement('button');
source2Button.textContent = 'Fuente 2';
source2Button.style.margin = '5px';
source2Button.style.padding = '5px 10px';
switchingContainer.appendChild(source2Button);

const source3Button = document.createElement('button');
source3Button.textContent = 'Fuente 3';
source3Button.style.margin = '5px';
source3Button.style.padding = '5px 10px';
switchingContainer.appendChild(source3Button);

// Área de visualización de resultados
const resultsArea = document.createElement('div');
resultsArea.style.marginTop = '10px';
resultsArea.style.minHeight = '150px';
resultsArea.style.padding = '10px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.backgroundColor = '#f9f9f9';
switchingContainer.appendChild(resultsArea);

// Tres fuentes de datos
function createSource1() {
  return interval(1000).pipe(
    take(5),
    map((val) => `Fuente 1: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '#c8e6c9';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource2() {
  return interval(500).pipe(
    take(8),
    map((val) => `Fuente 2: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '#bbdefb';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource3() {
  return of('Fuente 3: A', 'Fuente 3: B', 'Fuente 3: C').pipe(
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '#ffccbc';
    })
  );
}

// Eventos de clic de botón
const source1Click$ = fromEvent(source1Button, 'click').pipe(map(() => 1));

const source2Click$ = fromEvent(source2Button, 'click').pipe(map(() => 2));

const source3Click$ = fromEvent(source3Button, 'click').pipe(map(() => 3));

// Fusionar clics de botón
merge(source1Click$, source2Click$, source3Click$)
  .pipe(
    // Cambiar a fuente seleccionada
    switchMap((sourceId) => {
      // Limpiar área de resultados
      resultsArea.innerHTML = '';

      // Devolver fuente seleccionada
      switch (sourceId) {
        case 1:
          return createSource1();
        case 2:
          return createSource2();
        case 3:
          return createSource3();
        default:
          return of('No se seleccionó ninguna fuente');
      }
    })
  )
  .subscribe((value) => {
    // Mostrar resultado
    const item = document.createElement('div');
    item.textContent = value;
    item.style.padding = '5px';
    item.style.margin = '2px 0';
    item.style.backgroundColor = 'white';
    item.style.borderRadius = '3px';
    resultsArea.appendChild(item);
  });

// Mensaje inicial
const initialMessage = document.createElement('div');
initialMessage.textContent =
  'Haga clic en un botón para seleccionar una fuente de datos';
initialMessage.style.color = '#666';
resultsArea.appendChild(initialMessage);

```

## Fusión Condicional

Este es un ejemplo de combinar `merge` y `filter` para seleccionar fuentes de datos según condiciones.

```ts
import { merge, interval, fromEvent } from 'rxjs';
import {
  map,
  filter,
  takeUntil,
  withLatestFrom,
  startWith,
} from 'rxjs';

// Crear elementos de UI
const conditionalContainer = document.createElement('div');
conditionalContainer.innerHTML = '<h3>Fusión Condicional:</h3>';
document.body.appendChild(conditionalContainer);

// Configuración de filtros
const filterDiv = document.createElement('div');
filterDiv.style.marginBottom = '10px';
conditionalContainer.appendChild(filterDiv);

// Crear casillas de verificación
const slowCheck = document.createElement('input');
slowCheck.type = 'checkbox';
slowCheck.id = 'slowCheck';
slowCheck.checked = true;
filterDiv.appendChild(slowCheck);

const slowLabel = document.createElement('label');
slowLabel.htmlFor = 'slowCheck';
slowLabel.textContent = 'Fuente Lenta';
slowLabel.style.marginRight = '15px';
filterDiv.appendChild(slowLabel);

const fastCheck = document.createElement('input');
fastCheck.type = 'checkbox';
fastCheck.id = 'fastCheck';
fastCheck.checked = true;
filterDiv.appendChild(fastCheck);

const fastLabel = document.createElement('label');
fastLabel.htmlFor = 'fastCheck';
fastLabel.textContent = 'Fuente Rápida';
fastLabel.style.marginRight = '15px';
filterDiv.appendChild(fastLabel);

const clickCheck = document.createElement('input');
clickCheck.type = 'checkbox';
clickCheck.id = 'clickCheck';
clickCheck.checked = true;
filterDiv.appendChild(clickCheck);

const clickLabel = document.createElement('label');
clickLabel.htmlFor = 'clickCheck';
clickLabel.textContent = 'Eventos de Clic';
filterDiv.appendChild(clickLabel);

// Botón de detener
const stopButton = document.createElement('button');
stopButton.textContent = 'Detener';
stopButton.style.marginLeft = '15px';
filterDiv.appendChild(stopButton);

// Área de visualización de resultados
const conditionalResults = document.createElement('div');
conditionalResults.style.height = '200px';
conditionalResults.style.overflowY = 'auto';
conditionalResults.style.padding = '10px';
conditionalResults.style.border = '1px solid #ddd';
conditionalResults.style.backgroundColor = '#f9f9f9';
conditionalContainer.appendChild(conditionalResults);

// Tres fuentes de datos
// 1. Fuente lenta (cada 1 segundo)
const slow$ = interval(1000).pipe(map((val) => ({ type: 'slow', value: val })));

// 2. Fuente rápida (cada 300 milisegundos)
const fast$ = interval(300).pipe(map((val) => ({ type: 'fast', value: val })));

// 3. Eventos de clic
const click$ = fromEvent(document.body, 'click').pipe(
  map((event) => ({
    type: 'click',
    value: {
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY,
    },
  }))
);

// Monitorear estado de casilla de verificación
const slowEnabled$ = fromEvent(slowCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

const fastEnabled$ = fromEvent(fastCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

const clickEnabled$ = fromEvent(clickCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// Evento de detener
const stop$ = fromEvent(stopButton, 'click');

// Fusión condicional
merge(
  // Combinar fuente lenta con estado habilitado
  slow$.pipe(
    withLatestFrom(slowEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Combinar fuente rápida con estado habilitado
  fast$.pipe(
    withLatestFrom(fastEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Combinar fuente de clic con estado habilitado
  click$.pipe(
    withLatestFrom(clickEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  )
)
  .pipe(takeUntil(stop$))
  .subscribe((event) => {
    // Mostrar resultado
    const item = document.createElement('div');

    switch (event.type) {
      case 'slow':
        item.textContent = `Fuente Lenta: ${event.value}`;
        item.style.color = '#1b5e20';
        break;
      case 'fast':
        item.textContent = `Fuente Rápida: ${event.value}`;
        item.style.color = '#0d47a1';
        break;
      case 'click':
        const clickValue = event.value as { x: number; y: number };
        item.textContent = `Clic: X=${clickValue.x}, Y=${clickValue.y}`;
        item.style.color = '#bf360c';
        break;
    }

    item.style.padding = '3px';
    item.style.margin = '2px 0';
    conditionalResults.prepend(item); // Mostrar más nuevo arriba
  });

```

## Resumen de Elección de Operadores de Combinación

| Propósito | Operador | Características |
|------|--------------|------|
| Siempre sincronizar múltiples últimos valores | `combineLatest` | Siempre combinar último valor de cada Observable |
| Obtener todo junto después de completación | `forkJoin` | Emitir solo último valor (una vez) |
| Procesar en orden sincrónicamente | `zip` | Combinar uno de cada Observable y emitir |
| Referenciar otros últimos valores en disparador | `withLatestFrom` | Adjuntar último valor de stream secundario cuando stream principal emite |

## Resumen

Los operadores de combinación son herramientas poderosas para combinar múltiples fuentes de datos en un solo stream. Al seleccionar el operador apropiado, los flujos de datos asíncronos complejos pueden expresarse de manera concisa y declarativa.

### Puntos Clave para Dominar Operadores de Combinación

1. **Seleccione el operador correcto para su caso de uso**: Cada operador está optimizado para un caso de uso específico. Elija el operador apropiado para su propósito.
2. **Comprenda cuándo emitir**: El comportamiento de los operadores de combinación depende en gran medida de cuándo se emiten los valores. Es importante comprender el tiempo de emisión de cada operador.
3. **Consideraciones de Manejo de Errores**: Considere el comportamiento cuando ocurre un error en parte del stream combinado (si todo falla o continúa parcialmente).
4. **Conocer condiciones de completación**: También es importante comprender cuándo el stream combinado completará, y completarlo explícitamente usando `takeUntil` o similar si es necesario.
5. **Aproveche la seguridad de tipos**: TypeScript le permite manejar operadores de combinación de manera type-safe. El beneficio de tipo es especialmente grande para combinaciones complejas.

Los operadores de combinación se pueden aprovechar en muchos escenarios prácticos como manejo de eventos de UI, múltiples solicitudes API, validación de formularios, etc. Dominar estos operadores le ayudará a desbloquear el verdadero poder de la programación reactiva en RxJS.

---
Siguiente, pasemos a [Manejo de Errores](/es/guide/error-handling/strategies) para aprender cómo escribir código RxJS más robusto!
