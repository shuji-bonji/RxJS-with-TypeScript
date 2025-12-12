---
description: Entenda as diferenças entre Promise e RxJS e aprenda a usá-los adequadamente. Promise especializa-se em processamento assíncrono único e executa imediatamente, enquanto RxJS é avaliado preguiçosamente, pode lidar com múltiplos valores, e pode ser cancelado e reutilizado. Este guia explica em detalhes as características de cada um e os critérios para selecioná-los através de comparações de código e casos de uso específicos.
---

# Diferenças entre Promise e RxJS

## Visão Geral

As principais ferramentas para lidar com processamento assíncrono em JavaScript/TypeScript são **Promise** e **RxJS (Observable)**. Embora ambos sejam às vezes usados para propósitos semelhantes, sua filosofia de design e casos de uso são bastante diferentes.

Esta página fornece informações para ajudá-lo a entender as diferenças entre Promise e RxJS e decidir qual usar.

## Diferenças Básicas

| Item | Promise | RxJS (Observable) |
|------|---------|-------------------|
| **Padronização** | Padrão JavaScript (ES6/ES2015) | Biblioteca de terceiros |
| **Valores emitidos** | Valor único | Zero ou mais valores múltiplos |
| **Avaliação** | Eager (executa imediatamente após criação) | Lazy (executa após inscrição) |
| **Cancelamento** | Não possível[^1] | Possível (`unsubscribe()`) |
| **Reutilização** | Não possível (resultado é apenas uma vez) | Possível (pode ser inscrito múltiplas vezes) |
| **Custo de aprendizado** | Baixo | Alto (requer entendimento de operadores) |
| **Casos de uso** | Processamento assíncrono único | Processamento de stream complexo |

[^1]: Embora o processamento baseado em Promise (como fetch) possa ser cancelado usando AbortController, a especificação Promise em si não tem função de cancelamento.

## Comparação de Código: Processamento Assíncrono Único

### Promise

```ts
// Promise executa imediatamente após a criação (Eager)
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json())
  .then(data => console.log(data))
  .catch(error => console.error(error));
```

Promise **inicia a execução no momento em que é definido** (avaliação Eager).

### RxJS

```ts
import { from } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

// Observable não executa até ser inscrito (Lazy)
const observable$ = from(fetch('https://jsonplaceholder.typicode.com/posts/1')).pipe(
  switchMap(response => response.json()), // response.json() retorna Promise, então use switchMap
  catchError(error => {
    console.error(error);
    return of(null);
  })
);

// A execução começa apenas quando inscrito
observable$.subscribe(data => console.log(data));
```

RxJS **não executa até que `subscribe()` seja chamado** (avaliação Lazy). Inscrever-se no mesmo Observable múltiplas vezes resulta em execuções independentes, e o processamento pode ser interrompido com `unsubscribe()`.

> [!TIP]
> **Diretrizes de uso prático**
> - Processamento imediato de uma única vez → Promise
> - Processamento a ser executado em um momento específico ou múltiplas vezes → RxJS

## Comparação de Código: Lidando com Múltiplos Valores

Uma das maiores diferenças entre Promise e RxJS é o número de valores que podem ser emitidos. Promise pode retornar apenas um único valor, enquanto RxJS pode emitir múltiplos valores ao longo do tempo.

### Impossível com Promise

Promise pode apenas **resolver uma vez**.

```ts
// Promise pode retornar apenas um único valor
const promise = new Promise(resolve => {
  resolve(1);
  resolve(2); // Este valor é ignorado
  resolve(3); // Este valor também é ignorado
});

promise.then(value => console.log(value));
// Saída: 1 (apenas o primeiro valor)
```

Uma vez que o valor é determinado pelo primeiro `resolve()`, as chamadas subsequentes de `resolve()` são ignoradas.

### Possível com RxJS

Observable **pode emitir valores qualquer número de vezes**.
```ts
import { Observable } from 'rxjs';

// Observable pode emitir múltiplos valores
const observable$ = new Observable(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  subscriber.complete();
});

observable$.subscribe(value => console.log(value));
// Saída: 1, 2, 3
```

Cada vez que `next()` é chamado, o valor é entregue ao assinante. Após emitir todos os valores, a conclusão é notificada com `complete()`. Esta característica permite o tratamento natural de dados de séries temporais em mudança, como comunicação em tempo real, dados de streaming e processamento contínuo de eventos.

> [!NOTE]
> **Exemplos de aplicação prática**
> - Receber mensagens WebSocket
> - Processamento sequencial de entrada de teclado
> - Streams de eventos do servidor (SSE)
> - Monitoramento contínuo de dados de sensores

## Comparação de Cancelamento

A capacidade de cancelar processamento assíncrono de longa duração ou desnecessário é importante do ponto de vista de gerenciamento de recursos e experiência do usuário. Há diferenças significativas nas capacidades de cancelamento entre Promise e RxJS.

### Promise (Não Cancelável)
Promise **não tem função de cancelamento padrão**.

```ts
const promise = new Promise(resolve => {
  setTimeout(() => resolve('Completo'), 3000);
});

promise.then(result => console.log(result));
// Não há maneira padrão de cancelar este processamento
```

Uma vez que a execução começa, ela não pode ser interrompida até a conclusão, o que pode causar vazamentos de memória e degradação de desempenho.

> [!WARNING]
> **Sobre AbortController**
> APIs Web como `fetch()` podem ser canceladas usando `AbortController`, mas isso não é um recurso da Promise em si, mas um mecanismo fornecido por APIs individuais. Não está disponível para todo processamento assíncrono.

### RxJS (Cancelável)

RxJS **pode ser cancelado a qualquer momento com `unsubscribe()`**.
```ts
import { timer } from 'rxjs';

const subscription = timer(3000).subscribe(
  () => console.log('Completo')
);

// Cancelar após 1 segundo
setTimeout(() => {
  subscription.unsubscribe(); // Cancelar
  console.log('Cancelado');
}, 1000);
// Saída: Cancelado ("Completo" não é exibido)
```

Cancelar a inscrição imediatamente interrompe o processamento em andamento e previne vazamentos de memória.

> [!TIP]
> **Casos de uso de cancelamento prático**
> - Cancelar requisições HTTP quando o usuário sai da tela
> - Descartar resultados de consulta de pesquisa antigos e processar apenas a consulta mais recente (`switchMap`)
> - Cancelar automaticamente todos os Observables quando o componente é destruído (padrão `takeUntil`)

## Qual Escolher

Se usar Promise ou RxJS depende da natureza do processamento e requisitos do projeto. Use os seguintes critérios como referência para selecionar a ferramenta apropriada.

### Quando Escolher Promise

Promise é adequado se as seguintes condições se aplicarem.

| Condição | Razão |
|------|------|
| Processamento assíncrono único | Uma requisição de API, uma leitura de arquivo, etc. |
| Fluxo de trabalho simples | `Promise.all`, `Promise.race` são suficientes |
| Projetos de pequena escala | Quer minimizar dependências |
| Usar apenas API padrão | Quer evitar bibliotecas externas |
| Código amigável para iniciantes | Quer reduzir custos de aprendizado |

#### Requisição de API Única:


```ts
interface User {
  id: number;
  name: string;
  email: string;
  username: string;
}

async function getUserData(userId: string): Promise<User> {
  const response = await fetch(`https://jsonplaceholder.typicode.com/users/${userId}`);
  if (!response.ok) {
    throw new Error('Falha ao recuperar dados do usuário');
  }
  return response.json();
}

// Exemplo de uso
getUserData('1').then(user => {
  console.log('Nome do usuário:', user.name);
  console.log('Email:', user.email);
});
```

Este código é um padrão típico para recuperar informações de usuário único. Usar `async/await` o torna tão legível quanto código síncrono. O tratamento de erros também pode ser unificado com `try/catch`, tornando-o simples e intuitivo.

#### Execução Paralela de Múltiplos Processos Assíncronos:

```ts
interface Post {
  id: number;
  userId: number;
  title: string;
  body: string;
}

async function loadAllData(): Promise<[User[], Post[]]> {
  const [users, posts] = await Promise.all([
    fetch('https://jsonplaceholder.typicode.com/users').then(r => r.json()),
    fetch('https://jsonplaceholder.typicode.com/posts').then(r => r.json())
  ]);
  return [users, posts];
}

// Exemplo de uso
loadAllData().then(([users, posts]) => {
  console.log('Número de usuários:', users.length);
  console.log('Número de posts:', posts.length);
});
```

`Promise.all()` permite executar múltiplas requisições de API em paralelo e aguardar todas elas completarem. Isso é muito conveniente para carregamento de dados inicial. Note que se mesmo uma falhar, todo o processo gera erro, mas sua simplicidade o torna fácil de entender e manter.

### Quando Escolher RxJS

RxJS é adequado se as seguintes condições se aplicarem.

| Condição | Razão |
|------|------|
| Processamento contínuo de eventos | Movimento do mouse, entrada de teclado, WebSocket, etc. |
| Processamento de stream complexo | Combinando e transformando múltiplas fontes de eventos |
| Cancelamento necessário | Quer controlar finamente o gerenciamento de recursos |
| Retry/Timeout | Quer tratamento de erros flexível |
| Projetos Angular | RxJS está integrado no framework |
| Dados em tempo real | Dados são continuamente atualizados |

#### Exemplo Concreto
```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'pesquisar: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);

// Pesquisa em tempo real (autocompletar)
if (!searchInput) throw new Error('Input de pesquisa não encontrado');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),              // Aguardar 300ms antes de processar
  distinctUntilChanged(),         // Processar apenas quando o valor mudar
  switchMap(query =>              // Executar apenas a requisição mais recente
    fetch(`https://api.github.com/search/users?q=${query}`).then(r => r.json())
  )
).subscribe(results => {
  console.log('Resultados da pesquisa:', results.items); // A API do GitHub armazena resultados na propriedade items
});
```

Este exemplo é um caso típico onde RxJS mostra seu verdadeiro valor. Ele monitora entrada do usuário, fornece um tempo de espera de 300ms para reduzir requisições desnecessárias, processa apenas quando o valor muda, e ao tornar apenas a requisição mais recente válida (`switchMap`), descarta automaticamente os resultados de requisições antigas.

> [!IMPORTANT]
> **Por que é difícil apenas com Promise**
> - Deve implementar manualmente debounce (controle de entrada contínua)
> - Deve gerenciar cancelamento de requisições antigas você mesmo
> - Esquecer de limpar event listeners causa vazamentos de memória
> - Deve rastrear múltiplos estados simultaneamente (timers, flags, gerenciamento de requisições)
>
> Com RxJS, todos esses podem ser realizados declarativamente em apenas algumas linhas.

## Interoperabilidade entre Promise e RxJS

Promise e RxJS não são mutuamente exclusivos e podem ser convertidos entre si e combinados. Isso é útil ao integrar código existente baseado em Promise em pipelines RxJS, ou inversamente quando você quer usar Observable em código existente baseado em Promise.

## Converter Promise para Observable

RxJS fornece múltiplas maneiras de converter uma Promise existente para Observable.

### Conversão por `from`

O método mais comum é usar `from`.

```ts
import { from } from 'rxjs';

// Criar Promise
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json());

// Converter para Observable com from()
const observable$ = from(promise);

observable$.subscribe({
  next: data => console.log('Dados:', data),
  error: error => console.error('Erro:', error),
  complete: () => console.log('Completo')
});
```

O resultado da Promise flui como Observable, e a conclusão também é chamada automaticamente.

### Conversão por `defer` (avaliação preguiçosa)

O `defer` atrasa a criação de uma Promise até que seja inscrito.

```ts
import { defer } from 'rxjs';

// Promise não é criada até subscribe
const observable$ = defer(() =>
  fetch('https://jsonplaceholder.typicode.com/posts/1').then(r => r.json())
);

// Criar nova Promise em cada subscribe
observable$.subscribe(data => console.log('1ª:', data));
observable$.subscribe(data => console.log('2ª:', data));
```

Este método é útil se você quiser criar uma nova Promise cada vez que se inscrever.

## Converter Observable para Promise

É possível pegar apenas um valor de um Observable e transformá-lo em Promise.

### `firstValueFrom` e `lastValueFrom`

As seguintes duas funções são recomendadas no RxJS 7 e posterior.

| Função | Comportamento |
|------|------|
| `firstValueFrom` | Retorna o primeiro valor como Promise |
| `lastValueFrom` | Retorna o último valor na conclusão como Promise |

```ts
import { of, firstValueFrom, lastValueFrom } from 'rxjs';
import { delay } from 'rxjs';

const observable$ = of(1, 2, 3).pipe(delay(1000));

// Obter primeiro valor como Promise
const firstValue = await firstValueFrom(observable$);
console.log(firstValue); // 1

// Obter último valor como Promise
const lastValue = await lastValueFrom(observable$);
console.log(lastValue); // 3
```

Se Observable completar antes do valor fluir, o padrão é um erro. Isso pode ser evitado especificando um valor padrão.

> [!WARNING]
> `toPromise()` está depreciado. Use `firstValueFrom()` ou `lastValueFrom()` em vez disso.

> [!TIP]
> **Diretrizes de seleção**
> - **`firstValueFrom()`**: Quando apenas o primeiro valor é necessário (por exemplo, resultado de autenticação de login)
> - **`lastValueFrom()`**: Quando o resultado final após processar todos os dados é necessário (por exemplo, resultado de agregação)

## Exemplo Prático: Combinando Ambos

No desenvolvimento real de aplicações, Promise e RxJS são frequentemente combinados.

> [!WARNING] Precauções Práticas
> Misturar Promise e Observable pode facilmente **cair em anti-padrões se os limites de design não forem claros**.
>
> **Problemas comuns:**
> - Torna-se não cancelável
> - Separação de tratamento de erros
> - `await` dentro de `subscribe` (especialmente perigoso)
> - Aquisição paralela dos mesmos dados com Promise e Observable
>
> Veja **[Capítulo 10: Anti-padrões de Mistura de Promise e Observable](/pt/guide/anti-patterns/promise-observable-mixing)** para detalhes.

### Submissão de Formulário e Chamadas de API

Exemplo de capturar evento de submissão de formulário do usuário no RxJS e enviá-lo ao servidor usando Fetch API (Promise).

```ts
import { fromEvent, from } from 'rxjs';
import { exhaustMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface FormData {
  username: string;
  email: string;
}

// Submissão de formulário baseada em Promise
async function submitForm(data: FormData): Promise<{ success: boolean }> {
  const response = await fetch('https://api.example.com/submit', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  });
  if (!response.ok) {
    throw new Error('Submissão falhou');
  }
  return response.json();
}

// Gerenciamento de stream de eventos com RxJS
const submitButton = document.createElement('button');
submitButton.id = 'submit-button';
submitButton.innerText = 'Enviar';
submitButton.style.padding = '10px 20px';
submitButton.style.margin = '10px';
document.body.appendChild(submitButton);
if (!submitButton) throw new Error('Botão de envio não encontrado');

fromEvent(submitButton, 'click').pipe(
  exhaustMap(() => {
    const formData: FormData = {
      username: 'testuser',
      email: 'test@example.com'
    };
    // Converter função Promise para Observable
    return from(submitForm(formData));
  }),
  catchError(error => {
    console.error('Erro de submissão:', error);
    return of({ success: false });
  })
).subscribe(result => {
  if (result.success) {
    console.log('Submissão bem-sucedida');
  } else {
    console.log('Submissão falhou');
  }
});
```

Cada vez que o botão de envio do formulário é clicado, um novo processo de submissão é iniciado, mas **ignora novas submissões durante a submissão**.

Neste exemplo, o uso de `exhaustMap` previne requisições duplicadas durante a transmissão.

### Autocompletar de Pesquisa

Exemplo de monitorar mudanças de valor do formulário de entrada e realizar pesquisas de API.

```ts
import { fromEvent, from } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface SearchResult {
  items: Array<{
    login: string;
    id: number;
    avatar_url: string;
  }>;
  total_count: number;
}

// Função de API baseada em Promise
async function searchAPI(query: string): Promise<SearchResult> {
  const response = await fetch(`https://api.github.com/search/users?q=${query}`);
  if (!response.ok) {
    throw new Error('Pesquisa falhou');
  }
  return response.json();
}

// Gerenciamento de stream de eventos com RxJS
const label = document.createElement('label');
label.innerText = 'pesquisar: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);
if (!searchInput) throw new Error('Input de pesquisa não encontrado');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),
  distinctUntilChanged(),
  switchMap(query => {
    // Converter função Promise para Observable
    return from(searchAPI(query));
  }),
  catchError(error => {
    console.error(error);
    return of({ items: [], total_count: 0 }); // Retornar resultado vazio em erro
  })
).subscribe(result => {
  console.log('Resultados da pesquisa:', result.items);
  console.log('Total:', result.total_count);
});
```

Neste exemplo, os seguintes controles são realizados:

- Aguardar 300ms para conclusão de entrada com `debounceTime(300)`
- `distinctUntilChanged()` para ignorar se o valor for o mesmo que o anterior
- `switchMap` para recuperar apenas os resultados de pesquisa mais recentes (requisições antigas são automaticamente canceladas)

> [!WARNING] Cuidado com anti-padrões
> O padrão de inscrever Observable em Promise pode causar vazamentos de memória e comportamento inesperado.
> <!-- TODO: Adicionar link para anti-padrão subscribe-in-promise quando disponível -->

> [!TIP]
> **Design por separação de responsabilidades**
>
> - **RxJS**: Encarregado de controle de eventos (debounce, switchMap, etc.)
> - **Promise**: Encarregado de requisições HTTP (async/await)
> - **`from()`**: Ponte entre ambos
>
> Usar cada tecnologia adequadamente melhora a legibilidade e manutenibilidade do código.

## Vantagens e Desvantagens

Cada tecnologia tem sua adequação e desvantagens.

### Promise
<div class="comparison-cards">

::: tip Benefícios
- Nenhuma dependência necessária pois é padrão JavaScript
- Código intuitivo e legível com `async/await`
- Baixo custo de aprendizado
- Processamento simples de tarefas únicas
:::

::: danger Desvantagens
- Não pode lidar com múltiplos valores
- Sem função de cancelamento
- Não adequado para processamento de stream contínuo
- Processamento de eventos complexos é difícil
:::

</div>

### RxJS
<div class="comparison-cards">

::: tip Benefícios
- Pode lidar com múltiplos valores ao longo do tempo
- Controle complexo possível com uma grande variedade de operadores
- Cancelamento (`unsubscribe`) é fácil
- Implementação flexível de tratamento de erros e retry
- Declarativo e testável
:::

::: danger Desvantagens
- Alto custo de aprendizado
- Requer bibliotecas
- Sobre-especificado para processos simples
- Depuração pode ser difícil
:::

</div>

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>

## Áreas Onde RxJS é Particularmente Ativo

RxJS é particularmente poderoso nas seguintes áreas. Ele pode resolver elegantemente requisitos complexos que são difíceis de alcançar apenas com Promise.

| Área | Exemplos | Comparação com Promise |
|------|----------|-------------------------|
| **Comunicação em Tempo Real** | WebSocket, SSE, chat, atualizações de preços de ações | Promise é apenas para comunicação única. Não adequado para processamento contínuo de mensagens |
| **Controle de Entrada do Usuário** | Autocompletar de pesquisa, validação de formulário | debounce, distinctUntilChanged, etc. são padrão |
| **Combinando Múltiplas Fontes** | Combinando condições de pesquisa × ordem de classificação × filtros | Pode ser descrito concisamente com combineLatest, withLatestFrom |
| **Suporte Offline** | PWA, monitoramento de status de rede, ressincronização automática | Controle de retry flexível com retry, retryWhen |
| **APIs de Streaming** | OpenAI, saída sequencial de tokens de resposta de IA | Pode processar dados contínuos em tempo real |
| **Controle de Cancelamento** | Interrompendo processos de longa duração, descartando requisições antigas | Pode cancelar imediatamente com unsubscribe() |

> [!NOTE]
> Para detalhes sobre o uso de RxJS, veja também [O que é RxJS - Casos de Uso](./what-is-rxjs.md#casos-de-uso).

## Resumo

| Propósito | Recomendado | Razão |
|------|------|------|
| Requisição HTTP única | Promise (`async/await`) | Simples, legível, API padrão |
| Processamento de evento de entrada do usuário | RxJS | Requer controle como debounce, distinct |
| Dados em tempo real (WebSocket) | RxJS | Pode lidar naturalmente com mensagens contínuas |
| Execução paralela de múltiplos processos assíncronos | Promise (`Promise.all`) | Promise é suficiente para execução paralela simples |
| Stream de evento contínuo | RxJS | Pode lidar com múltiplos valores ao longo do tempo |
| Processamento cancelável | RxJS | Cancelamento confiável com unsubscribe() |
| Aplicações simples | Promise | Baixo custo de aprendizado, poucas dependências |
| Aplicações Angular | RxJS | Padronizado integrado no framework |

### Política Básica
- **Use Promise se puder ser simples**
- **Use RxJS se processamento de stream complexo for necessário**
- **Combinar ambos também é eficaz** (ponte com `from()`)

RxJS é poderoso, mas você não precisa usar RxJS para todo processamento assíncrono. É importante usar a ferramenta certa na situação certa. Promise e RxJS são ambas ferramentas poderosas para lidar com processamento assíncrono, mas cada uma tem características diferentes.

- **Promise** é mais adequado para processamento assíncrono simples de uma única vez. Escolha Promise para processamento assíncrono básico por causa de seu baixo custo de aprendizado e boa compatibilidade com async/await.
- **RxJS** é poderoso quando lidar com múltiplos valores, processamento de eventos ou controle de fluxo de dados complexo é necessário. RxJS também é adequado quando controles avançados como cancel e retry são necessários.

No desenvolvimento real, é importante usar ambos adequadamente. Se necessário, você pode ser flexível convertendo Promise para Observable ou Observable para Promise.

> [!TIP] Próximos Passos
> - Aprenda mais sobre Observable em [O que é Observable](/pt/guide/observables/what-is-observable)
> - Aprenda como criar Observable em [Funções de Criação](/pt/guide/creation-functions/index)
> - Aprenda como converter e controlar Observables com [Operadores](/pt/guide/operators/index)
