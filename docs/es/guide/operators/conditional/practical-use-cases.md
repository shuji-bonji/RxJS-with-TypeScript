---
description: Se explicarán casos de uso prácticos de operadores condicionales de RxJS (iif, defer), incluyendo procesamiento de respaldo de API, estrategias de caché, selección dinámica de fuentes de datos y evaluación perezosa condicional. Se introducen patrones específicos de uso en situaciones que requieren ramas de procesamiento dinámico con ejemplos de código TypeScript. Aprenderás patrones de implementación que pueden aplicarse inmediatamente al desarrollo de aplicaciones reales.
---

# Casos de Uso Prácticos

Los operadores condicionales de RxJS se pueden utilizar para bifurcar y cambiar flujos según estados dinámicos.
En este capítulo, puedes experimentar los patrones de utilización de cada operador a través de código real funcional con UI.

## Selección de Diferentes Fuentes de Datos Basada en Condiciones

```ts
import { iif, of, EMPTY } from 'rxjs';
import { switchMap, tap, catchError, retry } from 'rxjs';

// Crear UI
const appContainer = document.createElement('div');
appContainer.innerHTML = '<h3>Aplicación de Selección de Fuente de Datos:</h3>';
document.body.appendChild(appContainer);

// Selección de opciones
const optionsDiv = document.createElement('div');
optionsDiv.style.marginBottom = '15px';
appContainer.appendChild(optionsDiv);

// Checkbox (modo offline)
const offlineCheck = document.createElement('input');
offlineCheck.type = 'checkbox';
offlineCheck.id = 'offlineMode';
optionsDiv.appendChild(offlineCheck);

const offlineLabel = document.createElement('label');
offlineLabel.htmlFor = 'offlineMode';
offlineLabel.textContent = 'Modo Offline';
offlineLabel.style.marginLeft = '5px';
optionsDiv.appendChild(offlineLabel);

// Entrada de ID de búsqueda
const idInput = document.createElement('input');
idInput.type = 'number';
idInput.placeholder = 'ID (1-10)';
idInput.min = '1';
idInput.max = '10';
idInput.value = '1';
idInput.style.marginLeft = '15px';
idInput.style.width = '80px';
optionsDiv.appendChild(idInput);

// Botón de búsqueda
const searchButton = document.createElement('button');
searchButton.textContent = 'Buscar';
searchButton.style.marginLeft = '10px';
optionsDiv.appendChild(searchButton);

// Área de resultados
const resultsArea = document.createElement('div');
resultsArea.style.padding = '15px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.borderRadius = '5px';
resultsArea.style.backgroundColor = '#f9f9f9';
resultsArea.style.minHeight = '150px';
appContainer.appendChild(resultsArea);

type User = {
  lastUpdated?: Date;
  fromCache?: boolean;
  id: number;
  name: string;
  email: string;
};
type ErrorResult = {
  error: boolean;
  message: string;
};

// Datos offline (caché)
const cachedData: Record<number, User> = {
  1: { id: 1, name: 'Taro Yamada', email: 'yamada@example.com' },
  2: { id: 2, name: 'Hanako Sato', email: 'sato@example.com' },
  3: { id: 3, name: 'Ichiro Suzuki', email: 'suzuki@example.com' },
};

// Obtener datos de API online (JSONPlaceholder)
function fetchUserFromApi(id: number) {
  console.log(`Obteniendo usuario ID ${id} de la API...`);

  // Endpoint de API real
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`Error HTTP: ${response.status}`);
        }
        return response.json();
      })
    ),
    tap(() => console.log('Llamada a API exitosa')),
    catchError((err) => {
      console.error('Llamada a API falló:', err);
      throw new Error('Solicitud a API falló');
    })
  );
}

// Obtener usuario del caché
function getUserFromCache(id: number) {
  console.log(`Obteniendo usuario ID ${id} del caché...`);

  return iif(
    () => id in cachedData,
    of({ ...cachedData[id], fromCache: true }),
    EMPTY.pipe(
      tap(() => {
        throw new Error('Usuario no encontrado en caché');
      })
    )
  );
}

// Clic en botón de búsqueda
searchButton.addEventListener('click', () => {
  const id = parseInt(idInput.value, 10);
  const isOffline = offlineCheck.checked;

  // Validación de entrada
  if (isNaN(id) || id < 1 || id > 10) {
    resultsArea.innerHTML =
      '<p style="color: red;">Por favor ingresa un ID válido (1-10)</p>';
    return;
  }

  // Mostrar carga
  resultsArea.innerHTML = '<p>Recuperando datos...</p>';

  // Seleccionar fuente de datos basada en modo offline
  iif(
    () => isOffline,
    getUserFromCache(id).pipe(
      catchError((err) => {
        console.error('Error de caché:', err);
        return of({ error: err.message });
      })
    ),
    fetchUserFromApi(id).pipe(
      retry(2), // Reintentar hasta 2 veces
      catchError((err) => {
        console.error('Error de API:', err);

        // Usar caché como respaldo si la API falla
        return getUserFromCache(id).pipe(
          catchError(() =>
            of({ error: 'Tanto la API online como el caché fallaron' })
          )
        );
      })
    )
  ).subscribe({
    next: (result: any) => {
      if ('error' in result) {
        resultsArea.innerHTML = `<p style="color: red;">Error: ${result.message}</p>`;
      } else {
        const source = result.fromCache
          ? '<span style="color: orange;">(del caché)</span>'
          : '<span style="color: green;">(de la API)</span>';

        resultsArea.innerHTML = `
          <h4>Información del Usuario ${source}</h4>
          <p><strong>ID:</strong> ${result.id}</p>
          <p><strong>Nombre:</strong> ${result.name}</p>
          <p><strong>Email:</strong> ${result.email}</p>
          ${
            result.lastUpdated
              ? `<p><small>Última actualización: ${new Date(
                  result.lastUpdated
                ).toLocaleString()}</small></p>`
              : ''
          }
        `;
      }
    },
    error: (err) => {
      resultsArea.innerHTML = `<p style="color: red;">Error: ${err.message}</p>`;
    },
  });
});

// Mensaje inicial
resultsArea.innerHTML = '<p>Haz clic en el botón para recuperar datos</p>';


```



## Bifurcación en Tiempo de Ejecución y Estrategias de Respaldo

En este ejemplo usando `iif`, la fuente de datos se cambia dinámicamente de "caché offline" y "API online" según las operaciones y estados del usuario.
Además, al combinar `catchError` y `retry`, se pueden definir reintentos y destinos de respaldo en caso de falla.

Es especialmente adecuado para los siguientes casos de uso:

- Soporte offline en entornos de red inestables
- Utilización de caché y cambio de solicitudes online
- Reintento automático y cambio a rutas alternativas en caso de falla de API

## Patrón de Optimización de Rendimiento

En escenarios más complejos, se pueden implementar patrones de adquisición de datos optimizados combinando operadores condicionales.

```ts
import { fromEvent, Observable, of, throwError, timer } from 'rxjs';
import {
  switchMap,
  catchError,
  map,
  tap,
  debounceTime,
  distinctUntilChanged,
  withLatestFrom,
  delay,
  startWith,
} from 'rxjs';

// Crear elementos UI
const optimizationContainer = document.createElement('div');
optimizationContainer.innerHTML = '<h3>Recuperación de Datos Condicional Avanzada:</h3>';
document.body.appendChild(optimizationContainer);

// UI de búsqueda
const searchInputGroup = document.createElement('div');
searchInputGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(searchInputGroup);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Ingresa ID de usuario (1-10)';
searchInput.value = '1';
searchInput.style.padding = '8px';
searchInput.style.width = '180px';
searchInputGroup.appendChild(searchInput);

const searchButton = document.createElement('button');
searchButton.textContent = 'Buscar';
searchButton.style.marginLeft = '10px';
searchButton.style.padding = '8px 16px';
searchInputGroup.appendChild(searchButton);

// Configuración de opciones
const optionsGroup = document.createElement('div');
optionsGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(optionsGroup);

const cacheCheckbox = document.createElement('input');
cacheCheckbox.type = 'checkbox';
cacheCheckbox.id = 'useCache';
cacheCheckbox.checked = true;
optionsGroup.appendChild(cacheCheckbox);

const cacheLabel = document.createElement('label');
cacheLabel.htmlFor = 'useCache';
cacheLabel.textContent = 'Usar Caché';
cacheLabel.style.marginRight = '15px';
optionsGroup.appendChild(cacheLabel);

const forceCheckbox = document.createElement('input');
forceCheckbox.type = 'checkbox';
forceCheckbox.id = 'forceRefresh';
optionsGroup.appendChild(forceCheckbox);

const forceLabel = document.createElement('label');
forceLabel.htmlFor = 'forceRefresh';
forceLabel.textContent = 'Forzar Actualización';
optionsGroup.appendChild(forceLabel);

// Área de visualización de resultados
const optimizedResults = document.createElement('div');
optimizedResults.style.padding = '15px';
optimizedResults.style.border = '1px solid #ddd';
optimizedResults.style.borderRadius = '5px';
optimizedResults.style.minHeight = '150px';
optimizedResults.style.backgroundColor = '#f9f9f9';
optimizationContainer.appendChild(optimizedResults);

// Gestión de caché
const cache = new Map<string, { data: any; timestamp: number }>();
const CACHE_EXPIRY = 30000; // 30 segundos

// Obtener datos de usuario de API real (JSONPlaceholder)
function fetchUserData(id: string, forceRefresh: boolean): Observable<any> {
  // ID inválido
  if (!id || isNaN(Number(id)) || Number(id) < 1 || Number(id) > 10) {
    return throwError(
      () => new Error('ID de usuario inválido: por favor ingresa un número entre 1 y 10')
    );
  }

  const cacheKey = `user-${id}`;
  const cachedItem = cache.get(cacheKey);
  const now = Date.now();

  // Verificar caché (dentro de expiración y no forzar actualización)
  if (
    !forceRefresh &&
    cachedItem &&
    now - cachedItem.timestamp < CACHE_EXPIRY
  ) {
    console.log(`Recuperado del caché: ${id}`);
    return of({
      ...cachedItem.data,
      fromCache: true,
    }).pipe(delay(100)); // Simular respuesta rápida
  }

  // Solicitud API real (JSONPlaceholder)
  console.log(`Recuperando datos de la API: ${id}`);
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`Error HTTP: ${response.status}`);
        }
        return response.json();
      })
    ),
    map((userData) => {
      const processedData = {
        id: userData.id,
        name: userData.name,
        email: userData.email,
        lastUpdated: now,
        fromCache: false,
      };

      // Guardar en caché
      cache.set(cacheKey, {
        data: processedData,
        timestamp: now,
      });

      return processedData;
    }),
    catchError((err) => {
      console.error('Error de API:', err);
      throw new Error('Solicitud a API falló');
    })
  );
}

// Monitorear cambios en condiciones de búsqueda
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map((event) => (event.target as HTMLInputElement).value.trim()),
  debounceTime(300),
  distinctUntilChanged()
);

// Monitorear cambios en configuración de caché
const useCache$ = fromEvent(cacheCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// Monitorear cambios en forzar actualización
const forceRefresh$ = fromEvent(forceCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(false)
);

// Evento de clic en botón de búsqueda
const searchClick$ = fromEvent(searchButton, 'click');

// Ejecutar búsqueda
searchClick$
  .pipe(
    // Obtener valor de entrada actual, configuración de caché, configuración de forzar actualización
    withLatestFrom(
      searchTerm$,
      useCache$,
      forceRefresh$,
      (_, term, useCache, forceRefresh) => ({
        term,
        useCache,
        forceRefresh,
      })
    ),
    tap(() => {
      // Mostrar inicio de búsqueda
      optimizedResults.innerHTML = '<p>Buscando...</p>';
    }),
    // Flujo condicional usando iif()
    switchMap(({ term, useCache, forceRefresh }) => {
      // Si el término de búsqueda está vacío
      if (!term) {
        return of({ error: 'Por favor ingresa un término de búsqueda' });
      }

      // Si el caché está deshabilitado
      if (!useCache) {
        return fetchUserData(term, true);
      }

      // Búsqueda normal (usar caché & forzar actualización si es necesario)
      return fetchUserData(term, forceRefresh);
    }),
    // Manejo de errores
    catchError((err) => {
      return of({ error: err.message });
    })
  )
  .subscribe({
    next: (result) => {
      if ('error' in result) {
        // Mostrar error
        optimizedResults.innerHTML = `
        <p style="color: red;">Error: ${result.error}</p>
      `;
      } else {
        // Mostrar datos
        const source = result.fromCache
          ? '<span style="color: orange;">(del caché)</span>'
          : '<span style="color: green;">(de la API)</span>';

        optimizedResults.innerHTML = `
        <h4>Información del Usuario ${source}</h4>
        <p><strong>ID:</strong> ${result.id}</p>
        <p><strong>Nombre:</strong> ${result.name}</p>
        <p><strong>Email:</strong> ${result.email}</p>
        ${
          result.lastUpdated
            ? `<p><small>Última actualización: ${new Date(
                result.lastUpdated
              ).toLocaleString()}</small></p>`
            : ''
        }
      `;
      }
    },
  });

// Mensaje inicial
optimizedResults.innerHTML =
  '<p>Ingresa un ID de usuario y haz clic en el botón de búsqueda</p>';

```


---

## Guía de Selección de Operadores

Muchos operadores condicionales se ven similares y pueden ser confusos, pero cada uno tiene un propósito de aplicación claro.
A continuación se muestra una comparación de flujos de decisión típicos y características.

## Cómo Elegir un Operador Condicional

| Operador | Caso de Uso | Características |
|------------|------------|------|
| `iif` | Seleccionar un flujo en tiempo de ejecución | Selecciona una de dos opciones basándose en una condición |
| `partition` | Separar un flujo en dos flujos basándose en una condición | Divide el flujo original en Verdadero/Falso basándose en una condición |
| `throwIfEmpty` | Detectar flujos vacíos | Lanza un error si no se emite ninguno de los valores |
| `defaultIfEmpty` | Usar valor por defecto si está vacío | Proporciona valor de respaldo si el flujo está vacío |

### Flujo de Decisión de Selección

1. **¿Hay dos opciones?**
   - Sí → Usa `iif`
   - No → Siguiente

2. **¿Quieres dividir el flujo?**
   - Sí → Usa `partition`
   - No → Siguiente

3. **¿Quieres lidiar con flujos vacíos?**
   - Sí → ¿Quieres tratar los flujos vacíos como errores?
     - Sí → `throwIfEmpty`
     - No → `defaultIfEmpty`
   - No → Siguiente

4. **¿Quieres simplemente filtrar valores basándose en una condición?**
   - Sí → Usa el operador `filter` (operador básico de filtrado)
   - No → Reexamina el propósito

## Resumen

Los operadores condicionales son herramientas poderosas para controlar el flujo de flujos y bifurcar el procesamiento basándose en condiciones específicas. Los puntos principales son los siguientes:

1. **Flujo reactivo basado en decisiones**: Los operadores condicionales se pueden usar para cambiar dinámicamente el procesamiento basándose en eventos o condiciones de datos.
2. **Manejo de errores mejorado**: Los operadores condicionales pueden servir como una parte importante de tu estrategia de manejo de errores, permitiendo el manejo elegante de casos excepcionales.
3. **Oportunidades de optimización**: La ejecución condicional evita el procesamiento innecesario y optimiza operaciones costosas, especialmente solicitudes de red y acceso a hardware.
4. **Flujos de aplicación complejos**: La lógica empresarial compleja y la gestión de estado se pueden expresar declarativamente combinando múltiples operadores condicionales.

Los operadores condicionales son especialmente valiosos al implementar manejo de errores, estrategias de caché, mecanismos de respaldo y patrones de ejecución condicional usando RxJS. Combinados con otros operadores, te permiten construir flujos de aplicación complejos de manera declarativa y con seguridad de tipos.
