---
description: O operador delayWhen controla dinamicamente o timing de atraso de cada valor com um Observable separado para alcançar processamento de atraso flexível de acordo com condições.
---

# delayWhen - Controle Dinâmico de Atraso

O operador `delayWhen` determina dinamicamente o tempo de atraso para cada valor **com um Observable individual**. Enquanto o operador `delay` fornece um atraso de tempo fixo, `delayWhen` pode aplicar um atraso diferente para cada valor.

## 🔰 Sintaxe Básica e Operação

Especifica uma função que retorna um Observable que determina o atraso para cada valor.

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
// Saída:
// A (após 1 segundo)
// C (após 1 segundo)
// B (após 2 segundos)
```

Neste exemplo, apenas o valor `'B'` terá um atraso de 2 segundos aplicado, os outros terão um atraso de 1 segundo.

[🌐 Documentação Oficial do RxJS - delayWhen](https://rxjs.dev/api/index/function/delayWhen)

## 💡 Exemplos de Uso Típicos

- **Atraso baseado em valor**: Alterar atraso com base em prioridade ou tipo
- **Atraso baseado em eventos externos**: Aguardar interação do usuário ou conclusão de outros streams
- **Atraso condicional**: Atrasar apenas para um valor específico
- **Controle de timing assíncrono**: Aguardar resposta da API ou prontidão dos dados

## 🧪 Exemplo de Código Prático 1: Atraso por Prioridade

Este é um exemplo de controle de timing de processamento de acordo com a prioridade da tarefa.

```ts
import { from, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

// Criação de UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'delayWhen - Atraso baseado em prioridade';
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
  { id: 1, name: 'Tarefa A', priority: 'low' },
  { id: 2, name: 'Tarefa B', priority: 'high' },
  { id: 3, name: 'Tarefa C', priority: 'medium' },
  { id: 4, name: 'Tarefa D', priority: 'high' },
  { id: 5, name: 'Tarefa E', priority: 'low' }
];

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const time = now.toLocaleTimeString('pt-BR', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${time}] ${message}`;
  output.appendChild(logItem);
}

addLog('Processamento iniciado', '#e3f2fd');

from(tasks)
  .pipe(
    delayWhen(task => {
      // Definir tempo de atraso de acordo com a prioridade
      let delayTime: number;
      switch (task.priority) {
        case 'high':
          delayTime = 500;  // Alta prioridade: 0.5 segundos
          break;
        case 'medium':
          delayTime = 1500; // Média prioridade: 1.5 segundos
          break;
        case 'low':
          delayTime = 3000; // Baixa prioridade: 3 segundos
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
        `Processando ${task.name} (prioridade: ${task.priority})`,
        colors[task.priority]
      );
    },
    complete: () => {
      addLog('Todas as tarefas concluídas', '#e3f2fd');
    }
  });
```

- Tarefas de alta prioridade são processadas após 0.5 segundos
- Tarefas de média prioridade são processadas após 1.5 segundos, baixa prioridade após 3 segundos
- Realiza ordem de processamento de acordo com a importância da tarefa

## 🧪 Exemplo de Código Prático 2: Atraso Devido a Eventos Externos

Este é um exemplo de espera por um clique do usuário antes de emitir um valor.

```ts
import { of, fromEvent } from 'rxjs';
import { delayWhen, take, tap } from 'rxjs';

// Criação de UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'delayWhen - Espera de clique';
container2.appendChild(title2);

const button = document.createElement('button');
button.textContent = 'Clique para exibir próximo valor';
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

of('Mensagem 1', 'Mensagem 2', 'Mensagem 3')
  .pipe(
    tap(msg => {
      addLog2(`Aguardando: ${msg} (por favor, clique no botão)`);
      button.textContent = `Clique para exibir "${msg}"`;
    }),
    delayWhen(() => {
      // Atraso até que o evento de clique ocorra
      return fromEvent(button, 'click').pipe(take(1));
    })
  )
  .subscribe({
    next: msg => {
      clickCount++;
      addLog2(`✅ Exibido: ${msg}`);
      if (clickCount < 3) {
        button.disabled = false;
      } else {
        button.textContent = 'Concluído';
        button.disabled = true;
      }
    },
    complete: () => {
      addLog2('--- Todas as mensagens exibidas ---');
    }
  });
```

- Cada valor é emitido após aguardar um clique do usuário
- Controle de atraso disparado por eventos externos é possível
- Pode ser aplicado ao processamento de sequência interativa

## 🆚 Comparação com delay

```ts
import { of, timer } from 'rxjs';
import { delay, delayWhen } from 'rxjs';

// delay - atraso de tempo fixo
of(1, 2, 3)
  .pipe(delay(1000))
  .subscribe(console.log);
// Todos os valores atrasados em 1 segundo

// delayWhen - atraso diferente por valor
of(1, 2, 3)
  .pipe(
    delayWhen(value => timer(value * 1000))
  )
  .subscribe(console.log);
// 1 após 1 segundo, 2 após 2 segundos, 3 após 3 segundos
```

| Operador | Controle de Atraso | Caso de Uso |
|:---|:---|:---|
| `delay` | Tempo fixo | Atraso uniforme simples |
| `delayWhen` | Dinâmico (por valor) | Atraso condicional, espera de evento externo |

## ⚠️ Notas Importantes

### 1. Observable de Atraso é Gerado Novamente Cada Vez

```ts
// ❌ Exemplo ruim: Reutilizando mesma instância de Observable
const delayObs$ = timer(1000);
source$.pipe(
  delayWhen(() => delayObs$)  // Não funcionará a partir da 2ª vez
).subscribe();

// ✅ Bom exemplo: Gerar novo Observable cada vez
source$.pipe(
  delayWhen(() => timer(1000))
).subscribe();
```

### 2. Quando o Observable de Atraso Não Completa

```ts
import { of, NEVER } from 'rxjs';
import { delayWhen } from 'rxjs';

// ❌ Exemplo ruim: Retornar NEVER atrasa para sempre
of(1, 2, 3)
  .pipe(
    delayWhen(() => NEVER)  // Valores não serão emitidos
  )
  .subscribe(console.log);
// Nenhuma saída
```

Observable de atraso deve sempre emitir um valor ou completar.

### 3. Tratamento de Erros

Se ocorrer um erro dentro do Observable de atraso, todo o stream terá erro.

```ts
import { of, throwError, timer, delayWhen } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delayWhen(value => {
      if (value === 2) {
        return throwError(() => new Error('Erro de atraso'));
      }
      return timer(1000);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Erro:', err.message)
  });
// Saída: 1
// Erro: Erro de atraso
```

## 📚 Operadores Relacionados

- **[delay](./delay)** - Atraso de tempo fixo
- **[debounceTime](../filtering/debounceTime)** - Atraso após parada de entrada
- **[throttleTime](../filtering/throttleTime)** - Passar valor a cada período fixo
- **[timeout](./timeout)** - Controle de timeout

## ✅ Resumo

O operador `delayWhen` controla dinamicamente o timing de atraso para cada valor.

- ✅ Atrasos diferentes podem ser aplicados a cada valor
- ✅ Controle de atraso por eventos externos e Observable
- ✅ Ajustar timing de processamento com base em prioridade e tipo
- ⚠️ Observable de atraso deve ser gerado novamente cada vez
- ⚠️ Observable de atraso deve completar ou emitir um valor
- ⚠️ Tenha cuidado com tratamento de erros
