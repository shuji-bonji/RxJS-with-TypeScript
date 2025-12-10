---
description: Esta sección proporciona una descripción general de ajax y fromFetch, Funciones de Creación para comunicación HTTP en RxJS, las diferencias entre ellas y directrices para su uso.
---

# Funciones de Creación de Comunicación HTTP

RxJS proporciona Funciones de Creación para manejar la comunicación HTTP como Observable. Esta sección describe dos funciones, `ajax()` y `fromFetch()`, en detalle.

## ¿Qué son las Funciones de Creación de Comunicación HTTP?

Las Funciones de Creación de Comunicación HTTP son un conjunto de funciones que permiten manejar la comunicación con APIs externas y servidores como un flujo Observable. Al usar estas funciones, la comunicación HTTP asíncrona puede integrarse en la cadena de operadores de RxJS, y el manejo de errores y procesamiento de reintentos pueden describirse declarativamente.

### Características Principales

- **Comunicación HTTP declarativa**: Al tratar la comunicación HTTP como Observable, es posible el procesamiento declarativo usando operadores
- **Manejo de errores uniforme**: Manejo de errores uniforme con operadores como `catchError()` y `retry()`
- **Cancelable**: Las solicitudes pueden cancelarse con `unsubscribe()`
- **Integración con otros streams**: Combinar con otros Observables via `switchMap()`, etc.

## Lista de Funciones de Creación de Comunicación HTTP

| Función | Descripción | Tecnología Base | Usos Principales |
|------|------|-----------|---------|
| [ajax()](/es/guide/creation-functions/http-communication/ajax) | Comunicación HTTP basada en XMLHttpRequest | XMLHttpRequest | Soporte de navegadores legacy, monitoreo de progreso |
| [fromFetch()](/es/guide/creation-functions/http-communication/fromFetch) | Comunicación HTTP basada en Fetch API | Fetch API | Navegadores modernos, comunicación HTTP ligera |

## Comparación: ajax() vs fromFetch()

### Diferencias Básicas

```typescript
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { fromFetch } from 'rxjs/fetch';

// ajax() - Parsea automáticamente la respuesta
const ajax$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');
ajax$.subscribe(data => console.log(data));

// fromFetch() - Parsear respuesta manualmente
const fetch$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1').pipe(
  switchMap(response => response.json())
);
fetch$.subscribe(data => console.log(data));

interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}
```

### Tabla de Comparación de Características

| Característica | ajax() | fromFetch() |
|------|--------|-------------|
| Tecnología Base | XMLHttpRequest | Fetch API |
| Parseo JSON Automático | ✅ Soportado con `getJSON()` | ❌ Llamar `.json()` manualmente |
| Eventos de Progreso | ✅ Soportado | ❌ No soportado |
| Timeout | ✅ Soporte integrado | ❌ Requiere implementación manual |
| Detección Automática de Errores HTTP | ✅ Automáticamente error en 4xx/5xx | ❌ Requiere verificación manual de estado |
| Cancelación de Solicitud | ✅ Posible con unsubscribe() | ✅ Posible con unsubscribe() |
| Soporte IE11 | ✅ Soportado | ❌ Requiere polyfill |
| Tamaño del Bundle | Algo más grande | Más pequeño |

## Directrices de Uso

### Cuándo Elegir ajax()

1. **Se requiere soporte de navegadores legacy**
   - Cuando necesitas soportar navegadores antiguos como IE11

2. **Se requiere monitoreo de progreso**
   - Cuando quieres mostrar el progreso de carga/descarga de archivos

3. **Obtención simple de JSON**
   - Cuando quieres obtener JSON fácilmente con `getJSON()`

4. **Se necesita detección automática de errores**
   - Cuando quieres usar detección automática de errores por código de estado HTTP

### Cuándo Elegir fromFetch()

1. **Solo se soportan navegadores modernos**
   - Cuando solo soportas entornos donde la Fetch API está disponible

2. **Quiere reducir el tamaño del bundle**
   - Cuando una función de comunicación HTTP ligera es suficiente

3. **Quiere usar características de Fetch API**
   - Cuando quieres manipular objetos Request/Response directamente
   - Cuando quieres usarlo en un Service Worker

4. **Necesita control fino**
   - Cuando quieres personalizar el procesamiento de respuesta en detalle

## Ejemplos de Uso Práctico

### Patrón de Llamada API

```typescript
import { of, catchError, retry, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

// Patrón práctico usando ajax()
const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout después de 5 segundos
  retry(2), // Reintentar dos veces en caso de fallo
  catchError(error => {
    console.error('Error al obtener usuario:', error);
    return of(null); // Retornar null en caso de error
  })
);

fetchUser$.subscribe({
  next: user => {
    if (user) {
      console.log('Usuario:', user);
    } else {
      console.log('Fallo al obtener usuario');
    }
  }
});
```

### Patrón de Envío de Formulario

```typescript
import { fromEvent, switchMap, map } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Convertir evento submit de formulario a Observable
const form = document.querySelector('form') as HTMLFormElement;
const submit$ = fromEvent(form, 'submit').pipe(
  map(event => {
    event.preventDefault();
    const formData = new FormData(form);
    return Object.fromEntries(formData.entries());
  }),
  switchMap(data =>
    ajax.post('https://api.example.com/submit', data, {
      'Content-Type': 'application/json'
    })
  )
);

submit$.subscribe({
  next: response => console.log('Envío exitoso:', response),
  error: error => console.error('Error de envío:', error)
});
```

## Preguntas Frecuentes

### P1: ¿Debo usar ajax() o fromFetch()?

**R:** Recomendamos `fromFetch()` si solo se soportan navegadores modernos. Las razones son las siguientes:
- Fetch API es el último estándar web
- Tamaño de bundle pequeño
- Alta compatibilidad futura

Sin embargo, elige `ajax()` en los siguientes casos:
- Se requiere soporte de IE11
- Se requiere monitoreo de progreso
- La obtención simple de JSON es suficiente

### P2: ¿Cómo se manejan los errores HTTP (4xx, 5xx)?

**R:**
- **ajax()**: Código de estado HTTP mayor a 400 se trata automáticamente como error y se llama el callback `error`
- **fromFetch()**: Los errores HTTP aún activan el callback `next`. Necesitas verificar `response.ok` manualmente

### P3: ¿Cómo cancelo una solicitud?

**R:** Ambos pueden cancelarse con `unsubscribe()`.

```typescript
const subscription = ajax.getJSON('/api/data').subscribe(...);

// Cancelar después de 3 segundos
setTimeout(() => subscription.unsubscribe(), 3000);
```

## Próximos Pasos

Para uso detallado de cada función, consulta las siguientes páginas:

- [ajax() en detalle](/es/guide/creation-functions/http-communication/ajax) - Comunicación HTTP basada en XMLHttpRequest
- [fromFetch() en detalle](/es/guide/creation-functions/http-communication/fromFetch) - Comunicación HTTP basada en Fetch API

## Recursos de Referencia

- [Documentación Oficial RxJS - ajax](https://rxjs.dev/api/ajax/ajax)
- [Documentación Oficial RxJS - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/es/docs/Web/API/Fetch_API)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/es/docs/Web/API/XMLHttpRequest)
