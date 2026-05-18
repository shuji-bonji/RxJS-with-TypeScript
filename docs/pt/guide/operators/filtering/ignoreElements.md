---
description: "O operador ignoreElements é um operador de filtragem RxJS que ignora todos os valores e só passa por conclusões e erros. Isso é útil quando se está aguardando a conclusão do processo."
---

# ignoreElements - somente conclusões/erros são aprovados

O operador `ignoreElements` **ignora todos os valores** emitidos pelo Observable de origem e somente as **notificações de conclusão e de erro** são transmitidas.

## 🔰 Sintaxe básica e uso

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valor:', value), // Não chamado
  complete: () => console.log('Concluído')
});
// Saída: Concluído
```

**Fluxo de operação**:.
1. todos os 1, 2, 3, 4 e 5 são ignorados
2. Somente as notificações de conclusão são transmitidas para baixo

[🌐 Documentação oficial do RxJS - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Padrão de utilização típico.

- **Wait for process completion**: quando você não precisa do valor e só quer saber a conclusão.
- **Executar somente efeitos colaterais**: executar efeitos colaterais com tap e ignorar valores
- Tratamento de erros**: quando você deseja apenas capturar erros
- **Sincronização de sequências**: espera pela conclusão de vários processos

## Exemplo prático de código 1: aguardar a conclusão do processo de inicialização

Este é um exemplo de espera pela conclusão de vários processos de inicialização.

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs';

// UICriado
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Inicialização do aplicativo';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// Função para adicionar o registro de status
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// Processo de inicialização1: Conexão com o banco de dados
const initDatabase$ = from(['DBConexão...', 'Verificação da tabela...', 'DBPronto']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // Valores ignorados, somente a conclusão é notificada
);

// Processo de inicialização2: Arquivo de configuração sendo lido
const loadConfig$ = from(['Arquivo de configuração sendo lido...', 'Análise de configuração em andamento...', 'Aplicativo de configuração concluído']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// Processo de inicialização3: Autenticação do usuário
const authenticate$ = from(['Informações de autenticação sendo verificadas...', 'Verificação do token em andamento...', 'Autenticação concluída']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// Todos os processos de inicialização são executados.
addLog('Inicialização iniciada...', 'orange');

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
    completeMessage.textContent = '✅ Toda a inicialização foi concluída.！O aplicativo pode ser iniciado.';
    addLog('Aplicativo iniciado', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ Erro de inicialização: ${err.message}`;
  }
});
```

- É exibido um registro detalhado de cada processo de inicialização, mas os valores são ignorados.
- Quando todos os processos forem concluídos, será exibida uma mensagem de conclusão.

## Exemplo prático de código 2: Aguardando a conclusão do upload do arquivo

Este é um exemplo de exibição do progresso do upload de vários arquivos, mas apenas notificando a conclusão.

```ts
import { from, of, concat } from 'rxjs';
import { ignoreElements, tap, delay, mergeMap } from 'rxjs';

// UICriado
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Carregamento de arquivo';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Upload iniciado';
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

// Processo de upload de arquivo (com indicação de progresso)
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
    ignoreElements() // Valores de progresso ignorados, somente a conclusão é notificada
  );
}

button.addEventListener('click', () => {
  button.disabled = true;
  progressArea.innerHTML = '';
  result.style.display = 'none';

  // Todos os arquivos são carregados sequencialmente
  from(files).pipe(
    mergeMap(file => uploadFile(file), 2) // Máx.23 arquivos em paralelo
  ).subscribe({
    complete: () => {
      result.style.display = 'block';
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
      result.innerHTML = `
        <strong>✅ Upload concluído</strong><br>
        ${files.length}Um arquivo foi carregado...
      `;
      button.disabled = false;
    },
    error: err => {
      result.style.display = 'block';
      result.style.backgroundColor = '#ffebee';
      result.style.color = 'red';
      result.textContent = `❌ Erro: ${err.message}`;
      button.disabled = false;
    }
  });
});
```

- O progresso de cada arquivo é exibido, mas os valores de progresso em si não fluem para o downstream.
- Uma mensagem de conclusão é exibida quando todos os uploads tiverem sido concluídos.

## 🆚 Comparação com operadores semelhantes

### ignoreElements vs filter(() => false) vs take(0)

```ts
import { of } from 'rxjs';
import { ignoreElements, filter, take } from 'rxjs';

const source$ = of(1, 2, 3);

// ignoreElements: Ignorar todos os valores, a conclusão é transmitida
source$.pipe(
  ignoreElements()
).subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('ignoreElements: Concluído')
});
// Saída: ignoreElements: Concluído

// filter(() => false): Filtrar todos os valores, deixar a conclusão passar
source$.pipe(
  filter(() => false)
).subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('filter: Concluído')
});
// Saída: filter: Concluído

// take(0): Concluído imediatamente
source$.pipe(
  take(0)
).subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('take(0): Concluído')
});
// Saída: take(0): Concluído
```

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valor:', value), // Não chamado
  complete: () => console.log('Concluído')
});
// Saída: Concluído
```

**Recomendado**: use `ignoreElements()` se quiser ignorar intencionalmente todos os valores. A intenção do código ficará clara.

## 🔄 Manipulação de notificações de erro.

O `ignoreElements` ignora os valores, mas **passa as notificações de erro**.

```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('Ocorre um erro'))
).pipe(
  ignoreElements()
);

// Caso de sucesso
success$.subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('✅ Concluído'),
  error: err => console.error('❌ Erro:', err.message)
});
// Saída: ✅ Concluído

// Caso de erro
error$.subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('✅ Concluído'),
  error: err => console.error('❌ Erro:', err.message)
});
// Saída: ❌ Erro: Ocorre um erro
```

## ⚠️ Notas.

### 1. Os efeitos colaterais são realizados

IgnoreElements ignora os valores, mas os efeitos colaterais (por exemplo, tap) são executados.

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs';

of(1, 2, 3).pipe(
  tap(v => console.log('Efeitos colaterais:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('Valor:', v),
  complete: () => console.log('Concluído')
});
// Saída:
// Efeitos colaterais: 1
// Efeitos colaterais: 2
// Efeitos colaterais: 3
// Concluído
```

### Use com Observable

Quando usada com o Infinite Observable, a assinatura dura para sempre, pois a conclusão nunca chega.

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs';

// ❌ Caso ruim: Não concluído
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Concluído') // Não chamado
});

// ✅ Bom exemplo: take Concluído em
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('Concluído') // 5Chamado após um segundo
});
```

### 3. tipos em TypeScript

O valor de retorno de `ignoreElements` é do tipo `Observable<never>`.

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const numbers$: Observable<number> = of(1, 2, 3);

// ignoreElements O resultado de Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value é do tipo never portanto, esse bloco não é executado
    console.log(value);
  },
  complete: () => console.log('Somente conclusão')
});
```

### 4. se a conclusão não for garantida

Se a fonte não for concluída, o ignoreElements também não será concluído.

```ts
import { NEVER } from 'rxjs';
import { ignoreElements } from 'rxjs';

// ❌ NEVERnão será concluído nem emitirá um erro
NEVER.pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Concluído') // Não chamado
});
```

## 💡 Padrões práticos de combinação

### Padrão 1: sequência de inicialização

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valor:', value), // Não chamado
  complete: () => console.log('Concluído')
});
// Saída: Concluído

### Padrão 2: processo de limpeza

```

## 📚 Operadores relacionados.

- **[filter](. /filter)** - filtra valores com base em condições.
- **[take](. /take)** - apenas os primeiros N valores são obtidos.
- **[skip](. /skip)** - pula os primeiros N valores
- **[tap](. /utility/tap)** - executa uma ação secundária

## Resumo.

O operador ignoreElements ignora todos os valores e passa apenas por conclusões e erros.

- Ideal quando apenas a notificação de conclusão é necessária.
- Os efeitos colaterais (tap) são executados
- As notificações de erro também são transmitidas
- Intenção mais clara do que filter(() => false)`
- ⚠️ O Observable infinito não é concluído
- ⚠️ O tipo de valor de retorno é `Observable<never>`.
- ⚠️ O valor é completamente ignorado, mas os efeitos colaterais são executados
