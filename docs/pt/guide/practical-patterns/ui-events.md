---
description: "São explicados os padrões práticos para o processamento de eventos de UI usando o RxJS. Serão apresentados exemplos específicos de implementação de UI interativa, como controle de eventos de clique (aceleração, debounce, distinct), processamento de eventos de rolagem, arrastar e soltar, entrada de teclado (autocompletar) e suporte a multitoque, juntamente com o código TypeScript TypeScript. Você aprenderá padrões eficientes para lidar com eventos de alta frequência."
---

# Padrão de processamento de eventos da interface do usuário

O tratamento de eventos de interface do usuário é um dos desafios mais frequentes no desenvolvimento de front-end, e o RxJS permite implementar o tratamento de eventos complexos de forma declarativa e intuitiva.

Este artigo descreve padrões específicos de tratamento de eventos de UI que são necessários na prática, como clique, rolagem, arrastar e soltar e entrada de teclado.

## O que você aprenderá neste artigo.

- Controle de eventos de clique (acelerador, debounce, distinct)
- Manuseio eficiente de eventos de rolagem
- Implementação de arrastar e soltar
- Entrada de teclado e preenchimento automático
- Suporte multitoque
- Combinação de eventos compostos

```
Cliques do usuário: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Execução do processamento      Execução do processamento
```

> Este artigo pressupõe o conhecimento do [Capítulo 4: Operadores](. /operators/index.md) e pressupõe o conhecimento do seguinte. Em particular, é importante compreender os conceitos de `debounceTime`, `throttleTime` e `distinctUntilChanged`.

## Manipulação de eventos de clique.

### Problema: execução excessiva de processamento devido a uma série de cliques.

Cliques consecutivos em um botão podem resultar em processamento repetido, causando problemas de desempenho e bugs.

### Solução 1: controle com throttleTime

Processe somente o primeiro clique em um determinado período de tempo.


```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1Em um segundo.1Processado apenas uma vez
  ).subscribe(() => {
    console.log('Execução do processo de transmissão');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Durante a transmissão do formulário...');
  // APIChamadas, etc.
}
```

#### Fluxo de execução

```
Cliques do usuário: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Execução do processamento      Execução do processamento
Cliques do usuário: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Execução do processamento      Execução do processamento
```

> - Processa o **primeiro evento** e ignora os eventos subsequentes por um período de tempo
> Adequado quando o tempo real é importante (rolagem, redimensionamento, etc.)

### Solução 2: controle com debounceTime

Processar eventos após um determinado período de tempo depois que eles pararem.


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Inserção de palavras-chave de pesquisa...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Após a interrupção da entrada300msAguardar
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Execução da pesquisa:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Pesquisa em andamento...', query);
  // PesquisandoAPIRecuperar
}
```

#### Fluxo de execução

```
Entrada do usuário:  ●●●●●     ●●        ●●●●
                      |            |      |
debounceTime(300):   300ms       300ms  300msEm espera
                      |            |      |
                     Processamento         Processamento   Execução do processamento
Cliques do usuário: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Execução do processamento      Execução do processamento
```

> - Aguarde um determinado tempo após o **último evento** antes de processar
> - adequado para pesquisa, preenchimento automático e validação em tempo real

### Como usar o throttleTime vs. debounceTime

Cliques do usuário: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Execução do processamento      Execução do processamento

### Solução 3: deduplicação com distinctUntilChanged

Compare com o valor anterior e ignore o processamento se o mesmo valor for consecutivo.


```typescript
import { fromEvent, map, debounceTime, distinctUntilChanged } from 'rxjs';
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Inserção de palavras-chave de pesquisa...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  debounceTime(300),
  distinctUntilChanged() // Ignorado se o valor for o mesmo da última vez
).subscribe(query => {
  console.log('Execução da pesquisa:', query);
  performSearch(query);
});
```

#### Exemplo de execução

```typescript
// Entrada do usuário: "RxJS" → Backspace → "RxJS"
// distinctUntilChangedNenhum: 2Executar pesquisa uma vez
// distinctUntilChangedSim, se o valor for o mesmo que o anterior.: 1Executar a pesquisa apenas uma vez (mesmo valor, pular a segunda pesquisa)2A segunda busca é ignorada)
```

> [!TIP] ベストプラクティス

> Nas implementações de pesquisa e autocompletar, é recomendável usar os três conjuntos a seguir.
> 1. `debounceTime()` - espera pela parada da entrada.
> 2. `distinctUntilChanged()` - desduplicação
> 3. `switchMap()` - cancelar solicitações antigas

## Tratamento de eventos de rolagem

### Problema: disparo excessivo de eventos de rolagem

Os eventos de rolagem são disparados com muita frequência e podem causar problemas de desempenho se forem tratados como estão.

### Solução: reduza a frequência com o throttleTime.

```typescript
import { fromEvent, throttleTime, map } from 'rxjs';
const scrollContainer = document.createElement('div');
scrollContainer.id = 'scroll-container';
scrollContainer.style.width = '400px';
scrollContainer.style.height = '300px';
scrollContainer.style.overflow = 'auto';
scrollContainer.style.border = '1px solid #ccc';
scrollContainer.style.margin = '10px';
scrollContainer.style.padding = '10px';

// Add content to make it scrollable
scrollContainer.innerHTML = Array.from({ length: 100 }, (_, i) =>
  `<p>Item ${i + 1}</p>`
).join('');

document.body.appendChild(scrollContainer);

fromEvent(scrollContainer, 'scroll').pipe(
  throttleTime(100), // 100mspara1Processado apenas uma vez
  map(() => ({
    scrollTop: scrollContainer.scrollTop,
    scrollHeight: scrollContainer.scrollHeight,
    clientHeight: scrollContainer.clientHeight
  }))
).subscribe(({ scrollTop, scrollHeight, clientHeight }) => {
  // Cálculo da posição de rolagem
  const scrollPercentage = (scrollTop / (scrollHeight - clientHeight)) * 100;
  console.log(`Posição de rolagem: ${scrollPercentage.toFixed(1)}%`);

  // Rolagem infinita: 90%Carregar a próxima página depois de rolar mais de
  if (scrollPercentage > 90) {
    console.log('Carregamento da próxima página...');
    loadMoreItems();
  }
});

function loadMoreItems(): void {
  console.log('Aquisição de dados adicionais');
}
```

### Exemplo prático: detecção da direção da rolagem

```typescript
import { fromEvent, BehaviorSubject, throttleTime, map, pairwise, distinctUntilChanged } from 'rxjs';
type ScrollDirection = 'up' | 'down' | 'none';

const scrollDirection$ = new BehaviorSubject<ScrollDirection>('none');

// Create header element dynamically
const header = document.createElement('div');
header.id = 'header';
header.innerText = 'Cabeçalho (rolar para mostrar)/(oculto)';
header.style.position = 'fixed';
header.style.top = '0';
header.style.left = '0';
header.style.width = '100%';
header.style.padding = '20px';
header.style.background = '#333';
header.style.color = '#fff';
header.style.transition = 'transform 0.3s';
document.body.appendChild(header);

// Add scroll content
const scrollContent = document.createElement('div');
scrollContent.style.marginTop = '80px';
scrollContent.innerHTML = Array.from({ length: 100 }, (_, i) =>
  `<p>Conteúdo ${i + 1}</p>`
).join('');
document.body.appendChild(scrollContent);

fromEvent(window, 'scroll').pipe(
  throttleTime(100),
  map(() => window.scrollY),
  pairwise(), // Obter valores anteriores e atuais em pares
  map(([prev, curr]) => {
    if (curr > prev) return 'down';
    if (curr < prev) return 'up';
    return 'none';
  }),
  distinctUntilChanged() // Notificação somente quando a direção muda
).subscribe(direction => {
  scrollDirection$.next(direction);
  console.log('Direção de rolagem:', direction);

  // Mostra o cabeçalho/Alternar entre oculto e visível
  if (direction === 'down') {
    header.style.transform = 'translateY(-100%)';
  } else if (direction === 'up') {
    header.style.transform = 'translateY(0)';
  }
});
```

```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Inserção de palavras-chave de pesquisa...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Após a interrupção da entrada300msAguardar
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Execução da pesquisa:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Pesquisa em andamento...', query);
  // PesquisandoAPIRecuperar
}
```

> `pairwise()` é um operador conveniente que permite obter os valores anteriores e atuais em pares. Ele pode ser usado para determinar a direção da rolagem, aumentar/diminuir valores e calcular diferenças.

## Implementação de arrastar e soltar

### Problema: combinações complexas de eventos do mouse

Arrastar e soltar é uma combinação complexa de eventos mousedown → mousemove → mouseup.

### Solução: combinar vários Observable.


```typescript
import { fromEvent, merge, map, switchMap, takeUntil, tap } from 'rxjs';
interface Position {
  x: number;
  y: number;
}

const draggableElement = document.createElement('div');
draggableElement.id = 'draggable';
draggableElement.innerText = 'Arrastar.';
draggableElement.style.position = 'absolute';
draggableElement.style.left = '100px';
draggableElement.style.top = '100px';
draggableElement.style.width = '150px';
draggableElement.style.height = '150px';
draggableElement.style.padding = '20px';
draggableElement.style.background = '#4CAF50';
draggableElement.style.color = '#fff';
draggableElement.style.cursor = 'move';
draggableElement.style.userSelect = 'none';
draggableElement.style.display = 'flex';
draggableElement.style.alignItems = 'center';
draggableElement.style.justifyContent = 'center';
document.body.appendChild(draggableElement);

const mouseDown$ = fromEvent<MouseEvent>(draggableElement, 'mousedown');
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

// Obtém a posição do elemento no início do arrasto
let initialX = 0;
let initialY = 0;

mouseDown$.pipe(
  tap((event: MouseEvent) => {
    event.preventDefault();
    // Registra a posição atual do elemento
    const rect = draggableElement.getBoundingClientRect();
    initialX = rect.left;
    initialY = rect.top;

    // Diferença da posição do mouse no início do arrasto
    initialX = rect.left - event.clientX;
    initialY = rect.top - event.clientY;

    draggableElement.style.opacity = '0.7';
  }),
  switchMap(() =>
    // mousedownQuando,mousemoveComeça a monitorar o
    mouseMove$.pipe(
      map((event: MouseEvent): Position => ({
        x: event.clientX + initialX,
        y: event.clientY + initialY
      })),
      // mouseupoumouseleavepara encerrar o monitoramento
      takeUntil(
        merge(
          mouseUp$,
          fromEvent(document, 'mouseleave')
        ).pipe(
          tap(() => {
            draggableElement.style.opacity = '1';
          })
        )
      )
    )
  )
).subscribe((position: Position) => {
  // Mover elemento
  draggableElement.style.left = `${position.x}px`;
  draggableElement.style.top = `${position.y}px`;
});
```

#### Fluxo de eventos

```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1Em um segundo.1Processado apenas uma vez
  ).subscribe(() => {
    console.log('Execução do processo de transmissão');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Durante a transmissão do formulário...');
  // APIChamadas, etc.
}
```
Cliques do usuário: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Execução do processamento      Execução do processamento

### Exemplo prático: exibição de atraso ao passar o mouse

Cliques do usuário: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Execução do processamento      Execução do processamento

## Limpeza do evento

### Problema: evitar vazamentos de memória

A falha em cancelar corretamente a inscrição de ouvintes de eventos pode causar vazamentos de memória.

### Solução: limpar com takeUntil.

Cliques do usuário: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Execução do processamento      Execução do processamento


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Inserção de palavras-chave de pesquisa...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Após a interrupção da entrada300msAguardar
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Execução da pesquisa:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Pesquisa em andamento...', query);
  // PesquisandoAPIRecuperar
}
```

> - **Aplicar o `takeUntil` a todas as assinaturas de eventos**
> - **Dispare `destroy$` na destruição do componente**
> - **Os eventos globais (window, document) precisam de cuidados especiais**
> - **Não se esqueça de `unsubscribe()` quando estiver gerenciando subscrições explicitamente**

## Exemplos práticos de componentes da interface do usuário

### Implementação de rolagem infinita

Cliques do usuário: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Execução do processamento      Execução do processamento

> [!TIP] exhaustMapの活用

> O exhaustMap pode ser usado para ignorar novas solicitações até que a solicitação anterior seja concluída. Isso evita solicitações duplicadas devido a uma sequência de rolagem.

## Código de teste.

Exemplo de teste para manipulação de eventos da interface do usuário.

Cliques do usuário: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Execução do processamento      Execução do processamento

## Resumo.

O domínio dos padrões de manipulação de eventos da interface do usuário pode proporcionar uma experiência de usuário interativa e agradável.


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Inserção de palavras-chave de pesquisa...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Após a interrupção da entrada300msAguardar
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Execução da pesquisa:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Pesquisa em andamento...', query);
  // PesquisandoAPIRecuperar
}
```

> - **throttleTime**: processado apenas uma vez em um determinado período de tempo (rolagem, redimensionamento)
> - **debounceTime**: processado após a interrupção do evento (pesquisa, autocompletar)
> - **distinctUntilChanged**: deduplicação (ignorar valores idênticos)
> - **switchMap**: cadeia de eventos complexa (arrastar e soltar)
> - **takeUntil**: limpeza confiável (evita vazamentos de memória)


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Inserção de palavras-chave de pesquisa...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Após a interrupção da entrada300msAguardar
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Execução da pesquisa:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Pesquisa em andamento...', query);
  // PesquisandoAPIRecuperar
}
```

> - **Desempenho**: aceleração/debounce para evitar o processamento excessivo
> - **Usabilidade**: defina tempos de atraso apropriados (por exemplo, 300 ms)
> - **Acessibilidade**: suporte à operação do teclado
> - **Multidispositivo**: suporte para toque e mouse
> - **Limpeza**: o takeUntil garante a liberação da memória

## Próximas etapas.

Depois de dominar o padrão de tratamento de eventos da interface do usuário, você poderá passar para os padrões a seguir.

- [form-handling](. /form-handling.md) - validação em tempo real, integração de vários campos.
- [Chamadas de API](. /api-calls.md) - integração de eventos da interface do usuário e chamadas de API
- processamento de dados em tempo real](. /real-time-data.md)) - WebSocket, SSE.
- [estratégias de cache](. /caching-strategies.md) - cache de dados de eventos

## Seções relacionadas.

- [Capítulo 4: Operadores de filtragem](. /operators/filtering/) - debounceTime, throttleTime detalhes.
- [Capítulo 4: Operadores de transformação](. /operators/transformation/) - mais sobre switchMap, exhaustMap
- Capítulo 2: Observable](. /observables/what-is-observable.md)) - noções básicas de fromEvent

## Recursos de referência

- [RxJS official: fromEvent](https://rxjs.dev/api/index/function/fromEvent) - mais sobre fromEvent()
- [MDN: Touch events](https://developer.mozilla.org/ja/docs/Web/API/Touch_events) - Como usar eventos de toque.
- [Learn RxJS: debounceTime](https://www.learnrxjs.io/learn-rxjs/operators/filtering/debouncetime) - Exemplos práticos de debounceTime
